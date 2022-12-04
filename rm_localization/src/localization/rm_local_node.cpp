//
// Created by luotinkai on 2022/11/3.
//

#include "localization/rm_localization.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "rm_localization");
  ros::NodeHandle nh;
  RmLocalization rm_localization;
  ros::Rate loop_rate(100);
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}