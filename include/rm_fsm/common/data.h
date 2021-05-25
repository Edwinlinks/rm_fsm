//
// Created by luohx on 7/20/20.
//

#ifndef RM_FSM_COMMON_DATA_H_
#define RM_FSM_COMMON_DATA_H_

#include <ros/ros.h>
#include <rm_msgs/DbusData.h>
#include <nav_msgs/Odometry.h>
#include <rm_msgs/GimbalDesError.h>

#include "rm_fsm/referee/referee.h"
#include "rm_fsm/common/target_cost_function.h"
#include "rm_fsm/common/heat_limit.h"

namespace rm_fsm {

class Data {
 public:
  Data() = default;

  ros::Subscriber dbus_sub_;
  ros::Subscriber track_sub_;
  ros::Subscriber gimbal_des_error_sub_;
  ros::Subscriber odom_sub_;

  rm_msgs::DbusData dbus_data_;
  rm_msgs::TrackDataArray track_data_array_;
  rm_msgs::GimbalDesError gimbal_des_error_;
  nav_msgs::Odometry odom_;

  Referee *referee_{};

  void init(ros::NodeHandle nh) {
    referee_ = new Referee();
    // sub
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>("/dbus_data", 10, &Data::dbusDataCallback, this);
    track_sub_ = nh.subscribe<rm_msgs::TrackDataArray>("/track", 10, &Data::trackCallback, this);
    gimbal_des_error_sub_ =
        nh.subscribe<rm_msgs::GimbalDesError>("/error_des", 10, &Data::gimbalDesErrorCallback, this);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/odom", 10, &Data::odomCallback, this);
    // pub
    ros::NodeHandle root_nh;
    referee_->referee_pub_ = root_nh.advertise<rm_msgs::Referee>("/referee", 1);
    referee_->super_capacitor_pub_ = root_nh.advertise<rm_msgs::SuperCapacitor>("/super_capacitor", 1);
    referee_->init();
  }

  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) { dbus_data_ = *data; }
  void trackCallback(const rm_msgs::TrackDataArray::ConstPtr &data) { track_data_array_ = *data; }
  void gimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) { gimbal_des_error_ = *data; }
  void odomCallback(const nav_msgs::Odometry::ConstPtr &data) { odom_ = *data; }
};

}
#endif //RM_FSM_COMMON_DATA_H_