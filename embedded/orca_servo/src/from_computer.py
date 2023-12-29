#!/usr/bin/env python3

import rospy
import serial
from std_msgs.msg import Int32MultiArray

# Define the serial port and baud rate
ser = serial.Serial('/dev/ttyUSB0', 9600)

def serial_to_ros():
   # Create a publisher
   pub = rospy.Publisher('servo_command', Int32MultiArray, queue_size=10)

   # Initialize the node
   rospy.init_node('serial_to_ros', anonymous=True)

   # Loop until the node is shut down
   while not rospy.is_shutdown():
       # Read data from the serial port
       data = ser.read(16)

       # Convert the data to integers
       data_int = [int(byte) for byte in data]

       # Create a message
       msg = Int32MultiArray()
       msg.data = data_int

       # Publish the message
       pub.publish(msg)

       # Print the data to the screen
       print(data_int)

       # Sleep for a while
       rospy.sleep(0.1)

if __name__ == '__main__':
   try:
       serial_to_ros()
   except rospy.ROSInterruptException:
       pass