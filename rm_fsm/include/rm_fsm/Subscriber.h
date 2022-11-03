//
// Created by qiayuan on 2022/6/15.
//
#pragma once

#include "StateMachine.h"

#include <rm_common/referee/data.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/Referee.h>
#include <rm_msgs/TofRadarData.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

class Subscriber {
public:
  explicit Subscriber(StateMachineContext &fsm_context)
      : context_(fsm_context) {
    ros::NodeHandle nh;
    dbus_sub_ = nh.subscribe<rm_msgs::DbusData>(
        "/dbus_data", 10, &Subscriber::dbusCallback, this);
    left_radar_sub_ = nh.subscribe<rm_msgs::TofRadarData>(
        "/controllers/tof_radar_controller/left_tof_radar/data", 10,
        &Subscriber::leftRadarCallback, this);
    right_radar_sub_ = nh.subscribe<rm_msgs::TofRadarData>(
        "/controllers/tof_radar_controller/right_tof_radar/data", 10,
        &Subscriber::rightRadarCallback, this);
    lower_track_sub_ = nh.subscribe<rm_msgs::TrackData>(
        "/track", 10, &Subscriber::lowerTrackCallback, this);
    lower_gimbal_des_error_sub_ = nh.subscribe<rm_msgs::GimbalDesError>(
        "/controllers/lower_gimbal_controller/error", 10,
        &Subscriber::lowerGimbalDesErrorCallback, this);
    joint_state_sub_ = nh.subscribe<sensor_msgs::JointState>(
        "/joint_states", 10, &Subscriber::jointStateCallback, this);
  }

  rm_common::RefereeData
      referee_; // Only contains chassis power info for chassis sender;

  rm_msgs::DbusData dbus_;
  rm_msgs::GimbalDesError lower_gimbal_des_error_;
  rm_msgs::TofRadarData left_radar_, right_radar_;
  rm_msgs::TrackData lower_track_data_;
  double pos_lower_yaw_{}, pos_lower_pitch_{};

private:
  void dbusCallback(const DbusData::ConstPtr &data) {
    dbus_ = *data;
    context_.dbusUpdate(*data);
  }
  void lowerGimbalDesErrorCallback(const GimbalDesError::ConstPtr &data) {
    lower_gimbal_des_error_ = *data;
  }
  void lowerTrackCallback(const TrackData::ConstPtr &data) {
    lower_track_data_ = *data;
  }
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr &data) {
    pos_lower_pitch_ = data->position[3];
    pos_lower_yaw_ = data->position[6];
  }
  void leftRadarCallback(const TofRadarData::ConstPtr &data) {
    context_.leftRadarCB(*data);
    left_radar_ = *data;
  }
  void rightRadarCallback(const TofRadarData::ConstPtr &data) {
    context_.rightRadarCB(*data);
    right_radar_ = *data;
  }

  StateMachineContext &context_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber left_radar_sub_, right_radar_sub_;
  ros::Subscriber lower_track_sub_, upper_track_sub_;
  ros::Subscriber referee_sub_, game_robot_status_sub_;
  ros::Subscriber joint_state_sub_;
  ros::Subscriber lower_gimbal_des_error_sub_;
};
