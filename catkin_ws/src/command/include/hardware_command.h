#include <signal.h>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <command/HardwareCommand.h>
#include <command/HardwareState.h>
#include <command/MakeStop.h>

#define IDLE 0
#define FORWARD 1
#define TURN_LEFT 2
#define TURN_RIGHT 3

ros::Publisher pub;
ros::Subscriber sub;
ros::ServiceServer service;

int movement;
float sensor1, sensor2, sensor3;
float sensor_distance, turn_right_speed, turn_left_speed, forward_speed_motor1, forward_speed_motor2, forward_speed_motor3;
void callbackSub(const command::HardwareState::ConstPtr& msg);
void sigintHandler(int sig);