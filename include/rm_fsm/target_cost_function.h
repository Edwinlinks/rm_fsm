//
// Created by kiana on 2021/3/22.
//

#ifndef SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_
#define SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <rm_msgs/TrackData.h>
#include <rm_msgs/TrackDataArray.h>
#include <geometry_msgs/Twist.h>

class TargetCostFunction {
 public:
  explicit TargetCostFunction(ros::NodeHandle &nh);
  void input(rm_msgs::TrackDataArray track_data_array, bool only_attack_base = false);
  void decideId(rm_msgs::TrackDataArray track_data_array, bool only_attack_base = false);
  int output() const;
  static double calculateCost(rm_msgs::TrackData track_data);

 private:
  int id_{};
  double k_f_{};
  double timeout_;
  double calculate_cost_ = 1000000;
  double choose_cost_ = 1000000;
  double time_interval_{};
  ros::Time decide_old_target_time_, decide_new_target_time_;
};

#endif //SRC_RM_SOFTWARE_RM_FSM_INCLUDE_RM_FSM_TARGET_COST_FUNCTION_H_
