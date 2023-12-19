import rospy
from std_msgs.msg import Int32MultiArray, Float32
from geometry_msgs.msg import Quaternion

class ServoController:
    def __init__(self):
        rospy.init_node('servo_controller', anonymous=True)
        self.publisher_ = rospy.Publisher('servo_command', Int32MultiArray, queue_size=10)
        rospy.Subscriber('analog_values', Int32MultiArray, self.listener_callback1)
        rospy.Subscriber('quaternion', Quaternion, self.listener_callback2)
        rospy.Subscriber('height', Float32, self.listener_callback3)

    def send_command(self, command):
        msg = Int32MultiArray()
        msg.data = command
        self.publisher_.publish(msg)

    def listener_callback1(self, msg):
        rospy.loginfo('Analog values: %s' % msg.data)

    def listener_callback2(self, msg):
        rospy.loginfo('Quaternion: x=%s, y=%s, z=%s, w=%s' % (msg.x, msg.y, msg.z, msg.w))

    def listener_callback3(self, msg):
        rospy.loginfo('Height: %s' % msg.data)

def main():
    servo_controller = ServoController()

    # Send a command to the servos
    servo_controller.send_command([90, 90, 90])

    rospy.spin()

if __name__ == '__main__':
    main()