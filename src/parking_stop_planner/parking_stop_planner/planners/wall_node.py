import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleWallBot(Node):
    def __init__(self):
        super().__init__('turtle_wall_bot')
        
        # Hareket komutları göndermek için publisher
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Kaplumbağanın konumunu almak için subscriber
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        
        self.get_logger().info("Turtle Wall Bot başlatıldı. Duvarlar kontrol ediliyor...")

    def pose_callback(self, msg):
        vel_msg = Twist()
        
        # Turtlesim ekran sınırları genelde 0.0 ile 11.0 arasındadır.
        # Güvenli bölge olarak 1.0 ve 10.0 sınırlarını belirleyelim.
        
        if msg.x > 10.0 or msg.x < 1.0 or msg.y > 10.0 or msg.y < 1.0:
            # Duvara çok yakın! Dur ve dön.
            vel_msg.linear.x = 0.5 
            vel_msg.angular.z = 1.2
            self.get_logger().warn(f"Duvara yaklaşıldı! Konum: x={msg.x:.2f}, y={msg.y:.2f}")
        else:
            # Güvenli alan: Düz git
            vel_msg.linear.x = 2.0
            vel_msg.angular.z = 0.0
            
        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleWallBot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()