//
// Created by luohx on 7/20/20.
//

#ifndef RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
#define RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/ChassisCmd.h>
#include <rm_msgs/GimbalCmd.h>
#include <rm_msgs/ShootCmd.h>
#include <rm_msgs/GimbalDesError.h>
#include "rm_fsm/referee.h"
#include "rm_fsm/power_limit.h"
#include "rm_fsm/shooter_heat_limit.h"
#include "rm_fsm/target_cost_function.h"

template<typename T>
class FsmData {
 public:
  FsmData() = default;

  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber odom_sub_;

  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackDataArray track_data_array_;

  //chassis
  rm_msgs::ChassisCmd chassis_cmd_;
  geometry_msgs::Twist cmd_vel_;
  ros::Publisher vel_cmd_pub_;
  ros::Publisher chassis_cmd_pub_;
  PowerLimit *power_limit_{};

  //gimbal
  rm_msgs::GimbalCmd gimbal_cmd_;
  ros::Publisher gimbal_cmd_pub_;
  TargetCostFunction *target_cost_function_{};
  rm_msgs::GimbalDesError gimbal_des_error_;

  //shooter
  rm_msgs::ShootCmd shoot_cmd_;
  ros::Publisher shooter_cmd_pub_;
  ShooterHeatLimit *shooter_heat_limit_{};

  nav_msgs::Odometry odom_;

  Referee *referee_{};

  void init(ros::NodeHandle nh) {
    power_limit_ = new PowerLimit(nh);
    shooter_heat_limit_ = new ShooterHeatLimit();
    target_cost_function_ = new TargetCostFunction(nh);
    referee_ = new Referee();
    // sub
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(
        "/dbus_data", 10, &FsmData::dbusDataCallback, this);
    track_sub_ = nh.subscribe<rm_msgs::TrackDataArray>(
        "/track", 10, &FsmData::trackCallback, this);
    gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>(
        "/error_des", 10, &FsmData::gimbalDesErrorCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "/odom", 10, &FsmData::odomCallback, this);
    // pub
    ros::NodeHandle root_nh;
    vel_cmd_pub_ = root_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    chassis_cmd_pub_ = root_nh.advertise<rm_msgs::ChassisCmd>("/cmd_chassis", 1);
    gimbal_cmd_pub_ = root_nh.advertise<rm_msgs::GimbalCmd>("/cmd_gimbal", 1);
    shooter_cmd_pub_ = root_nh.advertise<rm_msgs::ShootCmd>("/cmd_shoot", 1);
    referee_->referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_->power_manager_pub_ = root_nh.advertise<rm_msgs::PowerManagerData>("/power_manager_data", 1);
    referee_->init(nh);
  }

  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }
  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) {
    track_data_array_ = *data;
  }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) {
    gimbal_des_error_ = *data;
  }
  void odomCallback(const nav_msgs::Odometry::ConstPtr &data) {
    odom_ = *data;
  }
};

#endif //RM_BASE_RM_DECISION_INCLUDE_FSM_CONTROL_FSM_DATA_H_
