#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int16.h>
#include <Encoder.h>

// Motor control pins
const int motor1PWM = 6;  // PWM pin for Motor 1 (Front left motor)
const int motor1DIR = 7;  // Direction pin for Motor 1

const int motor2PWM = 5;  // PWM pin for Motor 2 (Front right motor)
const int motor2DIR = 8;  // Direction pin for Motor 2

const int motor3PWM = 9;  // PWM pin for Motor 3 (Rear left motor)
const int motor3DIR = 10; // Direction pin for Motor 3

const int motor4PWM = 11; // PWM pin for Motor 4 (Rear right motor)
const int motor4DIR = 12; // Direction pin for Motor 4

// Encoder pins
const int encoderLeftA = 2;  // Encoder A pin for the front left wheel
const int encoderLeftB = 3;  // Encoder B pin for the front left wheel
const int encoderRightA = 20;  // Encoder A pin for the front right wheel
const int encoderRightB = 21;  // Encoder B pin for the front right wheel

// Define Encoder objects
Encoder encoderLeft(encoderLeftA, encoderLeftB);  // Encoder for Motor 1 (Front left)
Encoder encoderRight(encoderRightA, encoderRightB);  // Encoder for Motor 2 (Front right)

// Maximum speed (0 to 255)
const int maxSpeed = 200;

ros::NodeHandle nh;

// Publishers for encoder ticks
std_msgs::Int16 encoderLeftMsg;
std_msgs::Int16 encoderRightMsg;

ros::Publisher encoderLeftPub("encoderLeft_ticks", &encoderLeftMsg);
ros::Publisher encoderRightPub("encoderRight_ticks", &encoderRightMsg);

// Set speed function
void setSpeed(int speed) {
  analogWrite(motor1PWM, speed);
  analogWrite(motor2PWM, speed);
  analogWrite(motor3PWM, speed);
  analogWrite(motor4PWM, speed);
}

// Stop all motors
void stopMotors() {
  setSpeed(0);
  digitalWrite(motor1DIR, LOW);
  digitalWrite(motor2DIR, LOW);
  digitalWrite(motor3DIR, LOW);
  digitalWrite(motor4DIR, LOW);
  nh.loginfo("Motors stopped");
}

// Motor control functions (forward, backward, left, right)
void forward() {
  digitalWrite(motor1DIR, LOW);
  digitalWrite(motor2DIR, LOW);
  digitalWrite(motor3DIR, LOW);
  digitalWrite(motor4DIR, HIGH);
  setSpeed(maxSpeed);
  nh.loginfo("Moving forward");
}

void backward() {
  digitalWrite(motor1DIR, HIGH);
  digitalWrite(motor2DIR, HIGH);
  digitalWrite(motor3DIR, HIGH);
  digitalWrite(motor4DIR, LOW);
  setSpeed(maxSpeed);
  nh.loginfo("Moving backward");
}

void left() {
  digitalWrite(motor1DIR, HIGH);  // Left motors move backward
  digitalWrite(motor3DIR, HIGH);
  digitalWrite(motor2DIR, LOW);   // Right motors move forward
  digitalWrite(motor4DIR, HIGH);
  setSpeed(maxSpeed);
  nh.loginfo("Turning left");
}

void right() {
  digitalWrite(motor1DIR, LOW);   // Left motors move forward
  digitalWrite(motor3DIR, LOW);
  digitalWrite(motor2DIR, HIGH);  // Right motors move backward
  digitalWrite(motor4DIR, LOW);
  setSpeed(maxSpeed);
  nh.loginfo("Turning right");
}

// Callback for cmd_vel topic
void cmd_vel_callback(const geometry_msgs::Twist& msg) {
  float linear = msg.linear.x;
  float angular = msg.angular.z;

  if (linear > 0) {
    forward();
  } else if (linear < 0) {
    backward();
  } else if (angular > 0) {
    left();
  } else if (angular < 0) {
    right();
  } else {
    stopMotors();
  }
}

// Subscriber for cmd_vel
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", cmd_vel_callback);

void setup() {
  // Motor pins setup
  pinMode(motor1PWM, OUTPUT);
  pinMode(motor1DIR, OUTPUT);

  pinMode(motor2PWM, OUTPUT);
  pinMode(motor2DIR, OUTPUT);

  pinMode(motor3PWM, OUTPUT);
  pinMode(motor3DIR, OUTPUT);

  pinMode(motor4PWM, OUTPUT);
  pinMode(motor4DIR, OUTPUT);

  // Initialize ROS node
  nh.initNode();
  nh.subscribe(sub);

  // Advertise encoder tick topics
  nh.advertise(encoderRightPub);
  nh.advertise(encoderLeftPub);

  // Stop all motors initially
  stopMotors();
}

void loop() {
  // Read encoder values
  encoderLeftMsg.data = encoderLeft.read();
  encoderRightMsg.data = encoderRight.read();

  // Publish encoder ticks
  encoderLeftPub.publish(&encoderLeftMsg);
  encoderRightPub.publish(&encoderRightMsg);

  // Check for new messages
  nh.spinOnce();
}
