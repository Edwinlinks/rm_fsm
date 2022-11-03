//
// Created by luotinkai on 2022/11/3.
//

#include "rm_localization/rm_localization.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "tags_localization");

  ros::NodeHandle nh;

  // Create Nodes
  TagsLocalization tags_localization;

  ros::spin();

  // Release Nodes
  return 0;
}