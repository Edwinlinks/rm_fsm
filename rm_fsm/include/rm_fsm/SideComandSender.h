//
// Created by luotinkai on 2022/7/10.
//

#pragma once

#include <rm_common/decision/command_sender.h>
#include <rm_common/referee/referee.h>
#include <rm_msgs/GimbalDesError.h>
#include <rm_msgs/TrackData.h>
#include <ros/ros.h>

using namespace rm_common;

class SideCommandSender {
public:
  SideCommandSender(ros::NodeHandle &nh, const RefereeData &referee_data,
                    rm_msgs::TrackData &track_data,
                    rm_msgs::GimbalDesError &gimbal_des_error, double &pos_yaw,
                    double &pos_pitch)
      : gimbal_des_error_(gimbal_des_error), pos_yaw_(pos_yaw),
        pos_pitch_(pos_pitch) {
    ros::NodeHandle gimbal_nh(nh, "gimbal");
    gimbal_cmd_sender_ = new GimbalCommandSender(gimbal_nh, referee_data);
    ros::NodeHandle shooter_nh(nh, "shooter");
    shooter_cmd_sender_ =
        new ShooterCommandSender(shooter_nh, referee_data, track_data);
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
  GimbalCommandSender *gimbal_cmd_sender_;
  ShooterCommandSender *shooter_cmd_sender_;
  rm_msgs::GimbalDesError &gimbal_des_error_;
  double pitch_min_{}, pitch_max_{}, yaw_min_{}, yaw_max_{};
  double &pos_yaw_, &pos_pitch_;
  double yaw_direct_{1.}, pitch_direct_{1.};
};
