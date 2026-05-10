from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from smart_can_msgs.msg import (
    Rcthrtdata as RC_THRT_DATA,
    Autonomousbrakepedalcontrol as AUTONOMOUS_BrakePedalControl,
    Rcunittoomux as rc_unittoOmux,
    Autonomoushbmotorcontrol as AUTONOMOUS_HB_MotorControl,
    Autonomoussteeringmotcontrol as AUTONOMOUS_SteeringMotControl
)

class VehicleInterface:

    def __init__(self, node:Node):
        self.node = node
        self.current_speed_mps = 0.0
        self.steering_feedback = None
        self.emergency_active = False

        self.brake_pub = self.create_publisher(
            AUTONOMOUS_BrakePedalControl, '/beemobs/AUTONOMOUS_BrakePedalControl', 10)
        self.unittomux_pub = self.create_publisher(
            rc_unittoOmux, '/beemobs/rc_unittoOmux', 10)
        self.hb_pub = self.create_publisher(
            AUTONOMOUS_HB_MotorControl, '/beemobs/AUTONOMOUS_HB_MotorControl', 10)
        self.steer_pub = self.create_publisher(
            AUTONOMOUS_SteeringMotControl, '/beemobs/AUTONOMOUS_SteeringMot_Control', 10)
        

    def send_throttle(self, position):
        msg = RC_THRT_DATA()
        msg.rc_thrt_pedal_position = position
        msg.rc_thrt_pedal_press = 0 if position > 0 else 1
        self.thrt_pub.publish(msg)

    def send_brake(self, enable, pwm=0):
        msg = AUTONOMOUS_BrakePedalControl()
        msg.autonomous_brakemotor_voltage = 1
        msg.autonomous_brakepedalmotor_per = pwm if enable else 0
        msg.autonomous_brakepedalmotor_acc = 10000 if enable else 0
        msg.autonomous_brakepedalmotor_en = 1 if enable else 0
        self.brake_pub.publish(msg)

    def send_steering(self, pwm):
        msg = AUTONOMOUS_SteeringMotControl()
        msg.autonomous_steeringmot_en = 1
        msg.autonomous_steeringmot_pwm = pwm
        self.steer_pub.publish(msg)

    def send_ignition_and_gear(self):
        msg = rc_unittoOmux()
        msg.rc_ignition = 1
        msg.rc_selectiongear = 1
        msg.autonomous_emergency = 0
        self.unittomux_pub.publish(msg)
        self.get_logger().info("🟢 Ateşleme verildi ve vites ileri alındı.")

    def release_handbrake(self):
        msg = AUTONOMOUS_HB_MotorControl()
        msg.autonomous_hb_motor_pwm = 200
        msg.autonomous_hb_motstate = 1      # 1: indir
        msg.autonomous_hb_moten = 1
        self.hb_pub.publish(msg)

    def shutdown_procedure(self):
        self.get_logger().warn("Node kapatılıyor! Gaz kesiliyor, fren uygulanıyor...")
        self.send_throttle(0)
        self.send_brake(True, 100)


