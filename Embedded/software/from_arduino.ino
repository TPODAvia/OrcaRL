#include <ros/ros.h>
#include <Servo.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int32MultiArray.h>
#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>

Servo servo1;
Servo servo2;
Servo servo3;

#define NUM_SERVOS 30
int servoPins[NUM_SERVOS] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31}; 

ros::Publisher analog_pub;
ros::Publisher quaternion_pub;
ros::Publisher height_pub;

MPU9250_asukiaaa sensor;
Adafruit_BMP280 bmp;

void setup() {
  ros::init(argc, argv, "node_name");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("servo_command", 1000, callback);
  analog_pub = nh.advertise<std_msgs::Int32MultiArray>("analog_values", 1000);
  quaternion_pub = nh.advertise<geometry_msgs::Quaternion>("quaternion", 1000);
  height_pub = nh.advertise<std_msgs::Float32>("height", 1000);

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

  // Read sensor data
  sensor.accelUpdate();
  sensor.magUpdate();
  float ax = sensor.accelX();
  float ay = sensor.accelY();
  float az = sensor.accelZ();
  float mx = sensor.magX();
  float my = sensor.magY();
  float mz = sensor.magZ();

  // Compute quaternion
  geometry_msgs::Quaternion quaternion_msg;
  // This is a placeholder, replace this with your own quaternion computation
  // using sensor data and a complementary filter or DMP
  quaternion_msg.x = ax;
  quaternion_msg.y = ay;
  quaternion_msg.z = az;
  quaternion_msg.w = 1.0;

  // Read height distance
  float height = bmp.readAltitude(1013.25); // This value should be your local sea-level pressure
  std_msgs::Float32 height_msg;
  height_msg.data = height;

  // Publish data
  quaternion_pub.publish(quaternion_msg);
  height_pub.publish(height_msg);
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
