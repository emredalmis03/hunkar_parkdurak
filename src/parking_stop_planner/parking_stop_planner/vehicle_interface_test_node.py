import rclpy
from rclpy.node import Node
import time

from .vehicle_interface import VehicleInterface


class VehicleInterfaceTestNode(Node):
    def __init__(self):
        super().__init__('vehicle_interface_test_node')

        self.vehicle = VehicleInterface(self)

        self.timer = self.create_timer(2.0, self.run_test)
        self.step = 0

    def run_test(self):
        if self.step == 0:
            self.get_logger().info("Ignition + Gear")
            self.vehicle.send_ignition_and_gear()

        elif self.step == 1:
            self.get_logger().info("Handbrake release")
            self.vehicle.release_handbrake()

        elif self.step == 2:
            self.get_logger().info("Throttle veriliyor")
            self.vehicle.send_throttle(20)

        elif self.step == 3:
            self.get_logger().info("Direksiyon veriliyor")
            self.vehicle.send_steering(150)

        elif self.step == 4:
            self.get_logger().info("Fren yapiliyor")
            self.vehicle.send_brake(True, 100)

        elif self.step == 5:
            self.get_logger().info("Shutdown")
            self.vehicle.shutdown_procedure()
            rclpy.shutdown()

        self.step += 1


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInterfaceTestNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()