#include <ros.h>
#include <std_msgs/Float32MultiArray.h>


const int pwmPin1 = 9;
const int dirPin1 = 8;
const int pwmPin2 = 10;
const int dirPin2 = 7;


ros::NodeHandle nh;

void dd_vel_callback(const std_msgs::Float32MultiArray& msg) {
  // Get the motor values from the message
  float motor_values[4];
  motor_values[0] = msg.data[0];
  motor_values[1] = msg.data[0];
  motor_values[2] = msg.data[0];
  motor_values[3] = msg.data[0];
  
  // if (motor_values[0] >= 0){
  //   digitalWrite(13, HIGH);
  //   delay(1500);
  // } else {
  //   digitalWrite(13, LOW);
  // }

  int motor_speed_0 = (int) motor_values[0];
  int motor_speed_1 = (int) motor_values[1];



  if (motor_speed_0 > 0) {
    digitalWrite(dirPin1, HIGH); // Forward direction
    analogWrite(pwmPin1, motor_speed_0);
  } else if (motor_speed_0 < 0) {
    digitalWrite(dirPin1, LOW); // Reverse direction
    analogWrite(pwmPin1, -motor_speed_0);
  } else {
    digitalWrite(dirPin1, LOW); // Stop motor
    analogWrite(pwmPin1, 0);
  }

  // Determine the direction of motion for motor 2 based on the sign of the speed value
  if (motor_speed_1 > 0) {
    digitalWrite(dirPin2, LOW); // Forward direction
    analogWrite(pwmPin2, motor_speed_1);
  } else if (motor_speed_1 < 0) {
    digitalWrite(dirPin2, HIGH); // Reverse direction
    analogWrite(pwmPin2, -motor_speed_1);
  } else {
    digitalWrite(dirPin2, LOW); // Stop motor
    analogWrite(pwmPin2, 0);
  }




}

ros::Subscriber<std_msgs::Float32MultiArray> dd_vel_subscriber("/dd_controller/diff_drive_velocities", &dd_vel_callback);

void setup() {
  // Initialize the motor pins
  pinMode(8, OUTPUT);   //dir1
  pinMode(9, OUTPUT);   //pwm1
  pinMode(12, OUTPUT);  //dir2
  pinMode(11, OUTPUT);  //pwm2
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
