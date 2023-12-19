#include <ros/ros.h>
#include <Servo.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>

Servo servo1;
Servo servo2;
Servo servo3;

#define NUM_SERVOS 30
int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31}; 

ros::Publisher analog_pub;

void setup() {
  ros::init(argc, argv, "node_name");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("servo_command", 1000, callback);
  analog_pub = nh.advertise<std_msgs::Int32MultiArray>("analog_values", 1000);

  sensor.beginAccel();
  sensor.beginMag();
  if (!bmp.begin()) {
    ROS_ERROR("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
}

void loop() {
  ros::spinOnce();

  // Read analog values
  std_msgs::Int32MultiArray array;
  for (int i = 0; i < 16; i++) {
    array.data.push_back(analogRead(A0 + i));
  }
  
  // Publish the array
  analog_pub.publish(array);
}

void callback(const std_msgs::Int32& msg) {
  // Get the command from the message
  int command = msg.data;

  // Control the servos based on the command
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 10; j++) {
      int pin = servoPins[i*10 + j];
      servo1.attach(pin);
      servo1.write(command);
    }
  }
}
