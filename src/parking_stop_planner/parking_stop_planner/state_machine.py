"""Park-durak görevi için dengeli ve uygulanabilir state machine.

Bu modül yarışma senaryosundaki ana akışı 7 ana state ile yönetir:
FOLLOW_ROUTE -> HANDLE_STOP -> MERGE_BACK -> SEARCH_PARK -> PARK -> MISSION_COMPLETE
ve her noktadan FAIL_SAFE'e geçiş desteği sunar.
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional


class MissionState(Enum):
	FOLLOW_ROUTE = auto()
	HANDLE_STOP = auto()
	MERGE_BACK = auto()
	SEARCH_PARK = auto()
	PARK = auto()
	MISSION_COMPLETE = auto()
	FAIL_SAFE = auto()


class StopPhase(Enum):
	APPROACH_STOP = auto()
	ALIGN_AT_STOP = auto()
	FULL_STOP = auto()
	PASSENGER_SERVICE_WAIT = auto()
	READY_TO_EXIT = auto()


class MergePhase(Enum):
	PREPARE_EXIT = auto()
	GAP_CHECK = auto()
	JOIN_LANE = auto()


class ParkSearchPhase(Enum):
	ENTER_PARK_AREA = auto()
	SCAN_SLOTS = auto()
	SELECT_SLOT = auto()
	POSITION_FOR_PARK = auto()


class ParkPhase(Enum):
	PLAN_MANEUVER = auto()
	EXECUTE_MANEUVER = auto()
	FINE_ADJUST = auto()
	VERIFY_FINAL_POSE = auto()


class FailSafeReason(Enum):
	EMERGENCY_STOP = auto()
	SENSOR_FAULT = auto()
	COLLISION_RISK = auto()
	LOCALIZATION_UNSTABLE = auto()
	LANE_OR_DIRECTION_VIOLATION = auto()
	PATH_BLOCKED = auto()
	TIMEOUT = auto()


@dataclass
class StateInput:
	"""State machine'in her döngüde ihtiyaç duyduğu durum özeti."""

	dt_s: float = 0.1

	# Global güvenlik sinyalleri
	emergency_active: bool = False
	sensor_fault: bool = False
	collision_risk: bool = False
	localization_unstable: bool = False
	lane_or_direction_violation: bool = False
	path_blocked_too_long: bool = False
	timeout_exceeded: bool = False

	# Sürüş ve görev ilerleme sinyalleri
	speed_mps: float = 0.0
	goal_reached: bool = False
	at_stop_zone: bool = False
	stop_service_completed: bool = False
	at_parking_entry: bool = False

	# Durak davranışı
	stop_distance_m: float = 999.0
	stop_alignment_error_m: float = 999.0
	passenger_exchange_done: bool = False
	merge_lane_available: bool = False
	merged_into_route: bool = False

	# Park arama / park etme davranışı
	park_slot_found: bool = False
	selected_slot_valid: bool = False
	ready_for_parking_maneuver: bool = False
	park_maneuver_finished: bool = False
	park_pose_ok: bool = False


@dataclass
class StateOutput:
	"""State machine kararlarını üst katmana taşıyan komut özeti."""

	target_speed_mps: float = 0.0
	hold_position: bool = False
	request_stop_control: bool = False
	request_merge_control: bool = False
	request_park_search: bool = False
	request_parking_control: bool = False
	mission_complete: bool = False
	fail_safe_active: bool = False
	status_text: str = ""


@dataclass
class TransitionRecord:
	from_state: MissionState
	to_state: MissionState
	reason: str


@dataclass
class StateMachine:
	"""Park-durak görevi için ana state machine mekanizması."""

	state: MissionState = MissionState.FOLLOW_ROUTE

	stop_phase: StopPhase = StopPhase.APPROACH_STOP
	merge_phase: MergePhase = MergePhase.PREPARE_EXIT
	park_search_phase: ParkSearchPhase = ParkSearchPhase.ENTER_PARK_AREA
	park_phase: ParkPhase = ParkPhase.PLAN_MANEUVER

	stop_wait_timer_s: float = 0.0
	transition_log: List[TransitionRecord] = field(default_factory=list)
	fail_reason: Optional[FailSafeReason] = None

	# Ayarlanabilir eşikler
	stop_approach_threshold_m: float = 4.0
	stop_alignment_threshold_m: float = 0.35
	stop_speed_threshold_mps: float = 0.05
	passenger_wait_min_s: float = 2.0

	def update(self, data: StateInput) -> StateOutput:
		"""Bir kontrol döngüsü çalıştırır ve güncel state çıktısını döner."""
		if self._check_global_fail_safe(data):
			return self._run_fail_safe()

		if self.state == MissionState.FOLLOW_ROUTE:
			return self._run_follow_route(data)
		if self.state == MissionState.HANDLE_STOP:
			return self._run_handle_stop(data)
		if self.state == MissionState.MERGE_BACK:
			return self._run_merge_back(data)
		if self.state == MissionState.SEARCH_PARK:
			return self._run_search_park(data)
		if self.state == MissionState.PARK:
			return self._run_park(data)
		if self.state == MissionState.MISSION_COMPLETE:
			return self._run_mission_complete()

		return self._run_fail_safe()

	def _run_follow_route(self, data: StateInput) -> StateOutput:
		if data.at_stop_zone and not data.stop_service_completed:
			self._transition(MissionState.HANDLE_STOP, "stop zone reached")
			self.stop_phase = StopPhase.APPROACH_STOP
			self.stop_wait_timer_s = 0.0
			return self._run_handle_stop(data)

		if data.at_parking_entry:
			self._transition(MissionState.SEARCH_PARK, "parking entry reached")
			self.park_search_phase = ParkSearchPhase.ENTER_PARK_AREA
			return self._run_search_park(data)

		if data.goal_reached and data.stop_service_completed and data.park_pose_ok:
			self._transition(MissionState.MISSION_COMPLETE, "all objectives completed")
			return self._run_mission_complete()

		return StateOutput(
			target_speed_mps=4.0,
			status_text="FOLLOW_ROUTE: route takip ediliyor, görev noktasına ilerleniyor",
		)

	def _run_handle_stop(self, data: StateInput) -> StateOutput:
		if self.stop_phase == StopPhase.APPROACH_STOP:
			if data.stop_distance_m <= self.stop_approach_threshold_m:
				self.stop_phase = StopPhase.ALIGN_AT_STOP

		elif self.stop_phase == StopPhase.ALIGN_AT_STOP:
			if data.stop_alignment_error_m <= self.stop_alignment_threshold_m:
				self.stop_phase = StopPhase.FULL_STOP

		elif self.stop_phase == StopPhase.FULL_STOP:
			if data.speed_mps <= self.stop_speed_threshold_mps:
				self.stop_phase = StopPhase.PASSENGER_SERVICE_WAIT
				self.stop_wait_timer_s = 0.0

		elif self.stop_phase == StopPhase.PASSENGER_SERVICE_WAIT:
			self.stop_wait_timer_s += max(data.dt_s, 0.0)
			if (
				self.stop_wait_timer_s >= self.passenger_wait_min_s
				and data.passenger_exchange_done
			):
				self.stop_phase = StopPhase.READY_TO_EXIT

		elif self.stop_phase == StopPhase.READY_TO_EXIT:
			if data.merge_lane_available:
				self._transition(MissionState.MERGE_BACK, "stop completed, merge permitted")
				self.merge_phase = MergePhase.PREPARE_EXIT
				return self._run_merge_back(data)

		return StateOutput(
			target_speed_mps=0.8 if self.stop_phase == StopPhase.APPROACH_STOP else 0.0,
			hold_position=(
				self.stop_phase
				in (StopPhase.FULL_STOP, StopPhase.PASSENGER_SERVICE_WAIT, StopPhase.READY_TO_EXIT)
			),
			request_stop_control=True,
			status_text=f"HANDLE_STOP: {self.stop_phase.name}",
		)

	def _run_merge_back(self, data: StateInput) -> StateOutput:
		if self.merge_phase == MergePhase.PREPARE_EXIT:
			if data.merge_lane_available:
				self.merge_phase = MergePhase.GAP_CHECK

		elif self.merge_phase == MergePhase.GAP_CHECK:
			if data.merge_lane_available:
				self.merge_phase = MergePhase.JOIN_LANE

		elif self.merge_phase == MergePhase.JOIN_LANE:
			if data.merged_into_route:
				self._transition(MissionState.FOLLOW_ROUTE, "merged into traffic")
				return self._run_follow_route(data)

		return StateOutput(
			target_speed_mps=1.5 if self.merge_phase != MergePhase.JOIN_LANE else 2.5,
			request_merge_control=True,
			status_text=f"MERGE_BACK: {self.merge_phase.name}",
		)

	def _run_search_park(self, data: StateInput) -> StateOutput:
		if self.park_search_phase == ParkSearchPhase.ENTER_PARK_AREA:
			self.park_search_phase = ParkSearchPhase.SCAN_SLOTS

		elif self.park_search_phase == ParkSearchPhase.SCAN_SLOTS:
			if data.park_slot_found:
				self.park_search_phase = ParkSearchPhase.SELECT_SLOT

		elif self.park_search_phase == ParkSearchPhase.SELECT_SLOT:
			if data.selected_slot_valid:
				self.park_search_phase = ParkSearchPhase.POSITION_FOR_PARK

		elif self.park_search_phase == ParkSearchPhase.POSITION_FOR_PARK:
			if data.ready_for_parking_maneuver:
				self._transition(MissionState.PARK, "parking slot selected and pre-positioned")
				self.park_phase = ParkPhase.PLAN_MANEUVER
				return self._run_park(data)

		return StateOutput(
			target_speed_mps=1.2,
			request_park_search=True,
			status_text=f"SEARCH_PARK: {self.park_search_phase.name}",
		)

	def _run_park(self, data: StateInput) -> StateOutput:
		if self.park_phase == ParkPhase.PLAN_MANEUVER:
			self.park_phase = ParkPhase.EXECUTE_MANEUVER

		elif self.park_phase == ParkPhase.EXECUTE_MANEUVER:
			if data.park_maneuver_finished:
				self.park_phase = ParkPhase.FINE_ADJUST

		elif self.park_phase == ParkPhase.FINE_ADJUST:
			if data.park_pose_ok:
				self.park_phase = ParkPhase.VERIFY_FINAL_POSE

		elif self.park_phase == ParkPhase.VERIFY_FINAL_POSE:
			if data.park_pose_ok:
				self._transition(MissionState.MISSION_COMPLETE, "park completed and verified")
				return self._run_mission_complete()

		return StateOutput(
			target_speed_mps=0.5 if self.park_phase == ParkPhase.EXECUTE_MANEUVER else 0.0,
			request_parking_control=True,
			status_text=f"PARK: {self.park_phase.name}",
		)

	def _run_mission_complete(self) -> StateOutput:
		return StateOutput(
			target_speed_mps=0.0,
			hold_position=True,
			mission_complete=True,
			status_text="MISSION_COMPLETE: görev başarıyla tamamlandı",
		)

	def _run_fail_safe(self) -> StateOutput:
		reason = self.fail_reason.name if self.fail_reason else "UNKNOWN"
		return StateOutput(
			target_speed_mps=0.0,
			hold_position=True,
			fail_safe_active=True,
			status_text=f"FAIL_SAFE: güvenli duruş aktif ({reason})",
		)

	def _check_global_fail_safe(self, data: StateInput) -> bool:
		if data.emergency_active:
			return self._enter_fail_safe(FailSafeReason.EMERGENCY_STOP)
		if data.sensor_fault:
			return self._enter_fail_safe(FailSafeReason.SENSOR_FAULT)
		if data.collision_risk:
			return self._enter_fail_safe(FailSafeReason.COLLISION_RISK)
		if data.localization_unstable:
			return self._enter_fail_safe(FailSafeReason.LOCALIZATION_UNSTABLE)
		if data.lane_or_direction_violation:
			return self._enter_fail_safe(FailSafeReason.LANE_OR_DIRECTION_VIOLATION)
		if data.path_blocked_too_long:
			return self._enter_fail_safe(FailSafeReason.PATH_BLOCKED)
		if data.timeout_exceeded:
			return self._enter_fail_safe(FailSafeReason.TIMEOUT)
		return False

	def _enter_fail_safe(self, reason: FailSafeReason) -> bool:
		if self.state != MissionState.FAIL_SAFE:
			self.fail_reason = reason
			self._transition(MissionState.FAIL_SAFE, f"any-state fail-safe trigger: {reason.name}")
		return True

	def _transition(self, new_state: MissionState, reason: str) -> None:
		if self.state == new_state:
			return
		self.transition_log.append(
			TransitionRecord(from_state=self.state, to_state=new_state, reason=reason)
		)
		self.state = new_state


STATE_DESIGN: Dict[MissionState, Dict[str, object]] = {
	MissionState.FOLLOW_ROUTE: {
		"amac": "Normal sürüş ile görev noktalarına güvenli ve kural uyumlu ilerlemek",
		"davranislar": [
			"global rota takibi",
			"şerit/yön kuralına uygun sürüş",
			"durak veya park girişine kadar ilerleme",
		],
		"giris_kosullari": [
			"sistem başlangıcı",
			"MERGE_BACK tamamlandıktan sonra",
		],
		"cikis_kosullari": [
			"durak bölgesine giriş -> HANDLE_STOP",
			"park giriş bölgesine varış -> SEARCH_PARK",
		],
		"hata_durumlari": [
			"sensör arızası",
			"çarpışma riski",
			"lokalizasyon bozulması",
			"şerit/yön ihlali",
		],
		"alt_fazlar": [],
	},
	MissionState.HANDLE_STOP: {
		"amac": "Durağa kontrollü yaklaşma, doğru hizalanma, tam durma ve yolcu işlemi",
		"davranislar": [
			"durağa yaklaşma",
			"durak hizalama",
			"tam duruş",
			"yolcu alma/bırakma beklemesi",
		],
		"giris_kosullari": [
			"FOLLOW_ROUTE sırasında durak alanının algılanması",
		],
		"cikis_kosullari": [
			"yolcu işlemi tamamlandı ve güvenli çıkış mümkün -> MERGE_BACK",
		],
		"hata_durumlari": [
			"durakta yanlış konumlanma",
			"engelden dolayı güvenli duruşun bozulması",
			"bekleme timeout",
		],
		"alt_fazlar": [phase.name for phase in StopPhase],
	},
	MissionState.MERGE_BACK: {
		"amac": "Duraktan ayrıldıktan sonra trafiğe güvenli biçimde geri katılmak",
		"davranislar": [
			"çıkış hazırlığı",
			"boşluk/gap değerlendirme",
			"ana şeride katılım",
		],
		"giris_kosullari": [
			"HANDLE_STOP tamamlandı",
		],
		"cikis_kosullari": [
			"güvenli birleşim tamamlandı -> FOLLOW_ROUTE",
		],
		"hata_durumlari": [
			"uygun boşluk bulunamaması",
			"yanlış yönde katılım riski",
		],
		"alt_fazlar": [phase.name for phase in MergePhase],
	},
	MissionState.SEARCH_PARK: {
		"amac": "Park girişinden sonra uygun park slotunu bulup park öncesi konum almak",
		"davranislar": [
			"park alanına giriş",
			"slot tarama",
			"uygun slot seçimi",
			"park manevrası öncesi pozisyonlama",
		],
		"giris_kosullari": [
			"FOLLOW_ROUTE sırasında park giriş noktasına ulaşılması",
		],
		"cikis_kosullari": [
			"uygun slot seçimi ve ön pozisyon tamam -> PARK",
		],
		"hata_durumlari": [
			"uygun slot bulunamaması",
			"algı kaynaklı slot doğrulama hatası",
		],
		"alt_fazlar": [phase.name for phase in ParkSearchPhase],
	},
	MissionState.PARK: {
		"amac": "Park manevrasını uygulamak ve final pozunu doğrulamak",
		"davranislar": [
			"manevra planlama",
			"manevra yürütme",
			"ince düzeltmeler",
			"son poz doğrulama",
		],
		"giris_kosullari": [
			"SEARCH_PARK tamamlandı",
		],
		"cikis_kosullari": [
			"park tamamlandı ve doğrulandı -> MISSION_COMPLETE",
		],
		"hata_durumlari": [
			"park çizgisi/alanı taşması",
			"engel riski nedeniyle manevranın kesilmesi",
			"tekrar deneme limitinin aşılması",
		],
		"alt_fazlar": [phase.name for phase in ParkPhase],
	},
	MissionState.MISSION_COMPLETE: {
		"amac": "Görevi güvenli şekilde sonlandırmak",
		"davranislar": [
			"araç duruşunu koruma",
			"tamamlama durumunu raporlama",
		],
		"giris_kosullari": [
			"park görevi doğrulanarak tamamlandı",
		],
		"cikis_kosullari": [
			"normalde çıkış yok",
		],
		"hata_durumlari": [
			"harici sistem reseti dışında hata beklenmez",
		],
		"alt_fazlar": [],
	},
	MissionState.FAIL_SAFE: {
		"amac": "Kritik durumda güvenli duruşa geçip riski minimize etmek",
		"davranislar": [
			"hedef hız sıfırlama",
			"hareketi kesme",
			"hata nedenini raporlama",
		],
		"giris_kosullari": [
			"Any state -> FAIL_SAFE tetikleyici olay",
		],
		"cikis_kosullari": [
			"bu sürümde otomatik çıkış yok",
		],
		"hata_durumlari": [
			"emergency",
			"sensör arızası",
			"çarpışma riski",
			"lokalizasyon hatası",
			"kural ihlali",
		],
		"alt_fazlar": [],
	},
}


__all__ = [
	"FailSafeReason",
	"MergePhase",
	"MissionState",
	"ParkPhase",
	"ParkSearchPhase",
	"STATE_DESIGN",
	"StateInput",
	"StateMachine",
	"StateOutput",
	"StopPhase",
	"TransitionRecord",
]

