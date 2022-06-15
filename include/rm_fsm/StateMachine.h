//
// Created by luotinkai on 2022/2/27.
//

#pragma once

#include "StateMachine_sm.h"
#include "rm_fsm/common/fsm_data.h"
#include <realtime_tools/realtime_buffer.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/ros_utilities.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

class StateMachine {
public:
  explicit StateMachine(ros::NodeHandle &nh);

  static bool isRaw(rm_msgs::DbusData data_dbus_) {
    if (data_dbus_.s_r == rm_msgs::DbusData::MID)
      return true;
    else
      return false;
  }

  static bool isCruise(rm_msgs::DbusData data_dbus_) {
    if (data_dbus_.s_r == rm_msgs::DbusData::UP)
      return true;
    else
      return false;
  }

  void waitForUpdate() { context_.waitForUpdate(); }

  void changeVel() { auto_linear_x_ *= -1.; }

  void initRaw();

  void updateRandom(const ros::Time &time);

  void initCruise();

  void sendRawCommand(const ros::Time &time);

  void sendCruiseCommand(const ros::Time &time);

  void cruiseChassis();

  void cruiseShooter(SideCommandSender *);

  void cruiseGimbal(SideCommandSender *side_command_sender,
                    double yaw_direction, double pitch_direction);

  void setTrack(SideCommandSender *side_cmd_sender);

  void rawChassis();

  void rawGimbal(SideCommandSender *side_command_sender);

  void rawShooter(SideCommandSender *side_command_sender);

  // referee
  void chassisOutputOn();

  void gimbalOutputOn();

  void shooterOutputOn();

  void checkReferee(const ros::Time &time);

  // remote
  void remoteControlTurnOff();

  void remoteControlTurnOn();

  void cruiseRun();

  void rawRun();

  void checkSwitch(const ros::Time &time);

  ros::Time last_time_ = ros::Time::now();
  double rand_time_ = 0.8;

protected:
  StateMachineContext context_;
  rm_msgs::DbusData dbus_;
  FsmData fsm_data_;

  rm_common::CalibrationQueue *lower_trigger_calibration_{},
      *lower_gimbal_calibration_{}, *upper_gimbal_calibration_{},
      *catapult_calibration_{}, *upper_trigger_calibration_{};
  rm_common::ControllerManager controller_manager_;
  // calibrate
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  SideCommandSender *lower_cmd_sender_, *upper_cmd_sender_;
  // cruise
  double auto_linear_x_{};
  double safety_distance_{};
  std::default_random_engine random_;
  std::uniform_real_distribution<double> generator_{1.5, 2.0};
  // referee
  bool chassis_output_{}, shooter_output_{}, gimbal_output_{};
  // remote
  bool remote_is_open_{};

  ros::Subscriber dbus_sub_;
  ros::Subscriber left_radar_sub_;
  ros::Subscriber right_radar_sub_;

  void dbusCB(const rm_msgs::DbusData::ConstPtr &dbus_data) {
    dbus_ = *dbus_data;
    context_.dbusUpdate(*dbus_data);
  }

  void leftRadarCB(const rm_msgs::TofRadarDataConstPtr &radar_data) {
    if (radar_data->distance < safety_distance_ && auto_linear_x_ > 0) {
      context_.radarUpdate();
      last_time_ = ros::Time::now();
      rand_time_ = generator_(random_);
    }
  }

  void rightRadarCB(const rm_msgs::TofRadarDataConstPtr &radar_data) {
    if (radar_data->distance < safety_distance_ && auto_linear_x_ < 0) {
      context_.radarUpdate();
      last_time_ = ros::Time::now();
      rand_time_ = generator_(random_);
    }
  }
};
