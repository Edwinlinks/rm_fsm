//
// Created by luotinkai on 2022/3/2.
//

#pragma once

#include <random>
#include <rm_common/decision/command_sender.h>
#include <rm_common/referee/referee.h>
#include <rm_msgs/DbusData.h>
#include <rm_msgs/EventData.h>
#include <rm_msgs/GameRobotStatus.h>
#include <rm_msgs/GameStatus.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TofRadarData.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_listener.h>

class FsmData {
public:
  explicit FsmData();

  ~FsmData() = default;

  void lowerTrackCallback(const rm_msgs::TrackData::ConstPtr &data) {
    lower_track_data_ = *data;
  }

  void upperTrackCallback(const rm_msgs::TrackData::ConstPtr &data) {
    upper_track_data_ = *data;
  }

  void
  lowerGimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) {
    lower_gimbal_des_error_ = *data;
  }

  void
  upperGimbalDesErrorCallback(const rm_msgs::GimbalDesError::ConstPtr &data) {
    upper_gimbal_des_error_ = *data;
  }

  void robotStatusCallback(const rm_msgs::GameRobotStatusConstPtr &data) {
    game_robot_status_ = *data;
    referee_.referee_data_.game_robot_status_.chassis_power_limit_ =
        data->chassis_power_limit_;
    referee_.referee_data_.game_robot_status_.robot_id_ = data->robot_id_;
  }

  void gameStatusCallback(const rm_msgs::GameStatus::ConstPtr &data) {
    game_status_ = *data;
    referee_.referee_data_.game_status_.game_progress_ = data->game_progress_;
    referee_.referee_data_.game_status_.game_type_ = data->game_type_;
  }

  sensor_msgs::JointState joint_state_;
  rm_msgs::DbusData dbus_data_;
  ros::Subscriber lower_gimbal_des_error_sub_, upper_gimbal_des_error_sub_;
  double pos_x_{};
  double lower_yaw_{}, lower_pitch_{}, upper_yaw_{}, upper_pitch_{};
  rm_msgs::TrackData lower_track_data_, upper_track_data_;
  rm_msgs::GimbalDesError lower_gimbal_des_error_, upper_gimbal_des_error_;
  rm_common::Referee referee_;
  rm_msgs::GameRobotStatus game_robot_status_;
  rm_msgs::GameStatus game_status_;

private:
  void
  jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state) {
    joint_state_ = *joint_state;
    if (!joint_state_.position.empty()) {
      lower_yaw_ = joint_state_.position[6];
      lower_pitch_ = joint_state_.position[3];
      upper_yaw_ = joint_state_.position[11];
      upper_pitch_ = joint_state_.position[8];
    }
  }

  void dbusDataCallback(const rm_msgs::DbusData::ConstPtr &data) {
    dbus_data_ = *data;
  }

  ros::Subscriber joint_state_sub_;
  ros::Subscriber dbus_sub_;
  ros::Subscriber lower_track_sub_, upper_track_sub_, game_robot_status_sub_,
      game_status_sub_;
  ros::Subscriber referee_sub_;
  ros::NodeHandle nh_;
};

class SideCommandSender {
public:
  SideCommandSender(ros::NodeHandle &nh, rm_common::RefereeData &referee_data,
                    rm_msgs::GimbalDesError &gimbal_des_error, double &pos_yaw,
                    double &pos_pitch, rm_msgs::TrackData &track_data)
      : gimbal_des_error_(gimbal_des_error), pos_yaw_(pos_yaw),
        pos_pitch_(pos_pitch) {
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ =
        new rm_common::GimbalCommandSender(gimbal_nh, referee_data);
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ = new rm_common::ShooterCommandSender(
        shooter_nh, referee_data, track_data);
    ros::NodeHandle auto_nh(nh, "auto");
    XmlRpc::XmlRpcValue pitch_value, yaw_value;
    try {
      auto_nh.getParam("pitch", pitch_value);
      pitch_min_ = (double)(pitch_value[0]);
      pitch_max_ = (double)(pitch_value[1]);
      auto_nh.getParam("yaw", yaw_value);
      yaw_min_ = (double)(yaw_value[0]);
      yaw_max_ = (double)(yaw_value[1]);
    } catch (XmlRpc::XmlRpcException &e) {
      ROS_ERROR("%s", e.getMessage().c_str());
    }
  };
  rm_common::GimbalCommandSender *gimbal_cmd_sender_;
  rm_common::ShooterCommandSender *shooter_cmd_sender_;
  rm_msgs::GimbalDesError &gimbal_des_error_;
  double pitch_min_{}, pitch_max_{}, yaw_min_{}, yaw_max_{};
  double &pos_yaw_, &pos_pitch_;
  double yaw_direct_{1.}, pitch_direct_{1.};
};
