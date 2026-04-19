import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class TurtleNavigator(Node):
    def __init__(self):
        super().__init__('turtle_navigator')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Gitmek istediğimiz hedef nokta (Bunu ileride Reeds-Shepp'ten alacağız)
        self.goal_x = 6.0
        self.goal_y = 9.0
        self.tolerance = 0.1 # Hedefe ne kadar yaklaşırsak "vardık" sayalım?

    def pose_callback(self, msg):
        vel_msg = Twist()
        
        # 1. GÜVENLİK (Duvar Kontrolü - En yüksek öncelik)
        if msg.x > 10.5 or msg.x < 0.5 or msg.y > 10.5 or msg.y < 0.5:
            vel_msg.linear.x = 1.0  # Geri git
            vel_msg.angular.z = -1.0  # Sert dön
            self.get_logger().warn("DUVAR! Acil Kaçış!")
            self.publisher_.publish(vel_msg)
            return # Güvenlik tetiklendiyse hedefe gitmeyi bu döngülük iptal et
            
        # 2. HEDEFE YÖNELİM (Go-To-Goal Kontrolcüsü)
        distance = math.sqrt(pow((self.goal_x - msg.x), 2) + pow((self.goal_y - msg.y), 2))
        
        if distance >= self.tolerance:
            # Hedefe olan açıyı hesapla
            angle_to_goal = math.atan2(self.goal_y - msg.y, self.goal_x - msg.x)
            
            # Ne kadar dönmesi gerektiğini bul (Hedef açı - Mevcut açı)
            # Oransal (P) Kontrolcü kullanıyoruz: Fark ne kadar büyükse o kadar hızlı dön
            vel_msg.angular.z = 4.0 * (angle_to_goal - msg.theta)
            
            # Gaza bas (Mesafe kısaldıkça yavaşla)
            vel_msg.linear.x = 1.5 * distance
        else:
            # Hedefe vardık!
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            self.get_logger().info("Hedefe Ulaşıldı!")
            
        self.publisher_.publish(vel_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()