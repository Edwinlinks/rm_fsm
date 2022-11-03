//
// Created by luotinkai on 2022/2/27.
//

#pragma once

#include "StateMachine_sm.h"
#include "rm_fsm/SideComandSender.h"
#include "rm_fsm/Subscriber.h"

#include <random>
#include <realtime_tools/realtime_buffer.h>
#include <rm_common/decision/calibration_queue.h>
#include <rm_common/decision/command_sender.h>
#include <rm_common/decision/controller_manager.h>
#include <rm_common/ros_utilities.h>

using namespace rm_common;

class StateMachine {
public:
  explicit StateMachine(ros::NodeHandle &nh);

  static void info(const std::string &string) { ROS_INFO_STREAM(string); }
  void sendChassisCmd(bool is_auto, const DbusData &data) const;
  void sendGimbalCmd(bool is_auto, const DbusData &data,
                     SideCommandSender *side_command_sender) const;
  static void sendShooterCmd(bool is_auto, const DbusData &data,
                             SideCommandSender *side_command_sender);
  void calibrateChassisGimbal() const { chassis_gimbal_calibration_->reset(); }
  void calibrateShooter() const { shooter_calibration_->reset(); }
  void setTrack(SideCommandSender *side_cmd_sender) const;
  void reversal(bool enable);
  void catapult();
  void controllerUpdate();
  void update();
  rm_common::CalibrationQueue *chassis_gimbal_calibration_{},
      *shooter_calibration_{};
  rm_common::ControllerManager controller_manager_;
  rm_common::ChassisCommandSender *chassis_cmd_sender_;
  rm_common::Vel2DCommandSender *vel_2d_cmd_sender_;
  SideCommandSender *lower_cmd_sender_;

  StateMachineContext context_;
  Subscriber subscriber_;
  std::default_random_engine random_engine_;
  std::uniform_real_distribution<double> *random_generator_{};
  ros::Time begin_time_{ros::Time::now()};
  double auto_linear_vel_{}, safety_distance_{};
  double interval_time_{};
  bool enable_random_reversal_{};

protected:
private:
};
