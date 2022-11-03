//
// Created by luohx on 7/27/20.
//

#include "rm_fsm/StateMachine.h"

int main(int argc, char **argv) {
  std::string robot;
  ros::init(argc, argv, "rm_fsm");
  ros::NodeHandle fsm_nh("~");
  robot = getParam(fsm_nh, "robot_type", (std::string) "error");
  if (robot == "sentry") {
    ROS_INFO("Running sentry robot.");
  } else {
    ROS_ERROR("No robot type load");
    return 0;
  }
  StateMachine sm(fsm_nh);
  ROS_INFO("Enter sentry robot.");
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    sm.update();
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
