//
// Created by luotinkai on 2022/3/2.
//

#include "rm_fsm/common/fsm_data.h"

FsmData::FsmData() {
  joint_state_sub_ = nh_.subscribe<sensor_msgs::JointState>(
      "/joint_states", 10, &FsmData::jointStateCallback, this);
  dbus_sub_ = nh_.subscribe<rm_msgs::DbusData>(
      "/dbus_data", 10, &FsmData::dbusDataCallback, this);
  lower_track_sub_ = nh_.subscribe<rm_msgs::TrackData>(
      "/controllers/lower_gimbal_controller/track", 10,
      &FsmData::lowerTrackCallback, this);
  upper_track_sub_ = nh_.subscribe<rm_msgs::TrackData>(
      "/controllers/upper_gimbal_controller/track", 10,
      &FsmData::upperTrackCallback, this);
  lower_gimbal_des_error_sub_ = nh_.subscribe<rm_msgs::GimbalDesError>(
      "/controllers/lower_gimbal_controller/error_des", 10,
      &FsmData::lowerGimbalDesErrorCallback, this);
  upper_gimbal_des_error_sub_ = nh_.subscribe<rm_msgs::GimbalDesError>(
      "/controllers/upper_gimbal_controller/error_des", 10,
      &FsmData::upperGimbalDesErrorCallback, this);
  game_robot_status_sub_ = nh_.subscribe<rm_msgs::GameRobotStatus>(
      "/game_robot_status", 10, &FsmData::robotStatusCallback, this);
  game_status_sub_ = nh_.subscribe<rm_msgs::GameStatus>(
      "/game_status", 10, &FsmData::gameStatusCallback, this);
}
