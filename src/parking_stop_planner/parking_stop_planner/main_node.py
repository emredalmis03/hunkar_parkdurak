from enum import Enum, auto

import rclpy
from rclpy.node import Node

from .vehicle_interface import VehicleInterface


class MissionState(Enum):
	IDLE = auto()
	RUNNING = auto()
	EMERGENCY = auto()


class MainNode(Node):
	def __init__(self):
		super().__init__('main_node')

		self.state = MissionState.IDLE
		self.vehicle = VehicleInterface(self)

		self.create_timer(0.1, self.loop)

	def loop(self):
		if self.vehicle.emergency_active:
			self.state = MissionState.EMERGENCY
			self.vehicle.shutdown_procedure()
			return

		if self.state == MissionState.IDLE:
			self.get_logger().info('System is idle and waiting')
			return

		if self.state == MissionState.RUNNING:
			self.get_logger().info('System running (planner not integrated yet)')


def main(args=None):
	rclpy.init(args=args)
	node = MainNode()
	rclpy.spin(node)
	rclpy.shutdown()


if __name__ == '__main__':
	main()
