import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Quaternion

class ServoController(Node):
   def __init__(self):
       super().__init__('servo_controller')
       self.publisher_ = self.create_publisher(Int32MultiArray, 'servo_command', 10)
       self.subscription1 = self.create_subscription(Int32MultiArray, 'analog_values', self.listener_callback1, 10)
       self.subscription2 = self.create_subscription(Quaternion, 'quaternion', self.listener_callback2, 10)
       self.subscription3 = self.create_subscription(Float32, 'height', self.listener_callback3, 10)
       self.subscription1 # prevent unused variable warning
       self.subscription2 # prevent unused variable warning
       self.subscription3 # prevent unused variable warning

   def send_command(self, command):
       msg = Int32MultiArray()
       msg.data = command
       self.publisher_.publish(msg)

   def listener_callback1(self, msg):
       self.get_logger().info('Analog values: %s' % msg.data)

   def listener_callback2(self, msg):
       self.get_logger().info('Quaternion: x=%s, y=%s, z=%s, w=%s' % (msg.x, msg.y, msg.z, msg.w))

   def listener_callback3(self, msg):
       self.get_logger().info('Height: %s' % msg.data)

def main(args=None):
   rclpy.init(args=args)

   servo_controller = ServoController()

   # Send a command to the servos
   servo_controller.send_command([90, 90, 90])

   rclpy.spin(servo_controller)

   servo_controller.destroy_node()
   rclpy.shutdown()

if __name__ == '__main__':
   main()