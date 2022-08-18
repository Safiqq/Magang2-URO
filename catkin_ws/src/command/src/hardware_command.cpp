#include "hardware_command.h"

bool robot_status = false;
command::HardwareCommand pub_msg;

void callbackSub(const command::HardwareState::ConstPtr& msg) {
  ROS_INFO("data from sensor1: %lf", msg->sensor1);
  ROS_INFO("data from sensor2: %lf", msg->sensor2);
  ROS_INFO("data from sensor3: %lf", msg->sensor3);

  sensor1 = msg->sensor1;
  sensor2 = msg->sensor2;
  sensor3 = msg->sensor3;
}

bool callbackSrv(command::MakeStop::Request &req, command::MakeStop::Response &res) {
  robot_status = req.condition;
  ROS_INFO("robot status: %d", robot_status);
  if (!robot_status) ROS_INFO("robot stopped");
  return true;
}

void sigintHandler(int sig) {
  pub_msg.motor1 = 0;
  pub_msg.motor2 = 0;
  pub_msg.motor3 = 0;

  ROS_INFO("interrupt signal (%d) received", sig);
  ROS_INFO("robot will stand still");
  ros::shutdown();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "hardware_command");
  ros::NodeHandle nh;
  ros::Rate loop_rate(20);

  // load rosparam
  nh.getParam("sensor_distance", sensor_distance);
  nh.getParam("turn_right_speed", turn_right_speed);
  nh.getParam("turn_left_speed", turn_left_speed);
  nh.getParam("forward_speed_motor1", forward_speed_motor1);
  nh.getParam("forward_speed_motor2", forward_speed_motor2);
  nh.getParam("forward_speed_motor3", forward_speed_motor3);

  pub = nh.advertise<command::HardwareCommand>("/control/command/hardware", 1);
  sub = nh.subscribe("/arduino/state/hardware", 1, callbackSub);
  service = nh.advertiseService("/make_stop_service", callbackSrv);

  signal(SIGINT, sigintHandler);
  
  // The sensor is placed in a clockwise direction (sensor1 = left, sensor2 = front, sensor3 = right)
  // motor value (+) = turn right, (-) = turn left
  // sensor1: left
  // sensor2: middle
  // sensor3: right
  while (ros::ok()) {
    if (robot_status) {
      if (sensor1 >= sensor_distance && sensor2 >= sensor_distance && sensor3 >= sensor_distance) {
        ROS_INFO("robot will keep moving forward");
        movement = FORWARD;
      } else if (sensor1 < sensor_distance && sensor2 >= sensor_distance && sensor3 >= sensor_distance) {
        // there is an obstacle on the left, no obstacle ahead and right
        ROS_INFO("robot turning right");
        movement = TURN_RIGHT;
        // rotate in place to the right until no sensor detects obstacles
      } else if (sensor3 < sensor_distance && sensor1 >= sensor_distance && sensor2 >= sensor_distance) {
        // there is an obstacle on the right, no obstacle ahead and left
        ROS_INFO("robot turning left");
        movement = TURN_LEFT;
        // rotate in place to the left until no sensor detects obstacles
      } else if (sensor2 < sensor_distance && sensor1 >= sensor_distance && sensor3 >= sensor_distance) {
        // there are obstacle ahead, no obstacle left and right
        movement = TURN_RIGHT;
        ROS_INFO("obstacle ahead, robot turning right");
        // rotate in place to the right until no sensor detects obstacles
      } else if (sensor1 < sensor_distance && sensor2 < sensor_distance && sensor3 >= sensor_distance) {
        // there are obstacles ahead and left, no obstacle right
        movement = TURN_RIGHT;
        ROS_INFO("obstacle ahead and left, robot turning right");
      } else if (sensor1 < sensor_distance && sensor2 >= sensor_distance && sensor3 < sensor_distance) {
        // there are obstacles left and right, no obstacle ahead
        movement = TURN_RIGHT;
        ROS_INFO("obstacle ahead and left, robot turning right");
      } else if (sensor1 >= sensor_distance && sensor2 < sensor_distance && sensor3 < sensor_distance) {
        // there are obstacles ahead and left, no obstacle right
        movement = TURN_LEFT;
        ROS_INFO("obstacle ahead and left, robot turning right");
      } else {
        // there are obstacles in all sensors
        ROS_INFO("robot turn right");
        movement = TURN_RIGHT;
      }
    } else {
      // ROS_INFO("robot stopped");
      movement = IDLE;
    }
    switch (movement) {
      case IDLE:
        pub_msg.motor1 = 0;
        pub_msg.motor2 = 0;
        pub_msg.motor3 = 0;
        break;
      case FORWARD:
        pub_msg.motor1 = forward_speed_motor1;
        pub_msg.motor2 = forward_speed_motor2;
        pub_msg.motor3 = forward_speed_motor3;
        break;
      case TURN_LEFT:
        pub_msg.motor1 = turn_left_speed;
        pub_msg.motor2 = turn_left_speed;
        pub_msg.motor3 = turn_left_speed;
        break;
      case TURN_RIGHT:
        pub_msg.motor1 = turn_right_speed;
        pub_msg.motor2 = turn_right_speed;
        pub_msg.motor3 = turn_right_speed;
        break;
    }
    pub.publish(pub_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}