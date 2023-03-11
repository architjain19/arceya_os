#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

void dd_vel_callback(const std_msgs::Float32MultiArray& msg) {
  // Get the motor values from the message
  float motor_values[4];
  motor_values[0] = msg.data[0];
  motor_values[1] = msg.data[0];
  motor_values[2] = msg.data[0];
  motor_values[3] = msg.data[0];
  
  if (motor_values[0] >= 0){
    digitalWrite(13, HIGH);
    delay(1500);
  } else {
    digitalWrite(13, LOW);
  }
  // Actuate the motors
  // Assume the motors are connected to pins 5, 6, 9, and 10
  analogWrite(5, motor_values[0]);
  analogWrite(6, motor_values[1]);
  analogWrite(9, motor_values[2]);
  analogWrite(10, motor_values[3]);
}

ros::Subscriber<std_msgs::Float32MultiArray> dd_vel_subscriber("/dd_controller/diff_drive_velocities", &dd_vel_callback);

void setup() {
  // Initialize the motor pins
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);

  // Initialize the ROS node
  nh.initNode();
  
  // Subscribe to the "motor_values" topic
  nh.subscribe(dd_vel_subscriber);
}

void loop() {
  // Handle any incoming ROS messages
  nh.spinOnce();
}
