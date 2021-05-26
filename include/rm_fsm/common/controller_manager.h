//
// Created by astro on 2021/5/15.
//
#ifndef RM_FSM_COMMON_CONTROLLER_MANAGER_H_
#define RM_FSM_COMMON_CONTROLLER_MANAGER_H_

#include <ros/ros.h>
#include <controller_manager_msgs/LoadController.h>
#include <controller_manager_msgs/SwitchController.h>
#include <rm_common/ros_utilities.h>
namespace rm_fsm {
class ControllerManager {
 public:
  explicit ControllerManager(ros::NodeHandle &nh);

  bool loadControllers(const std::vector<std::string> &controllers);
  bool switchController(const std::vector<std::string> &start, const std::vector<std::string> &stop);

  bool startController(const std::string &controller) {
    std::vector<std::string> controllers;
    controllers.push_back(controller);
    return startController(controllers);
  }
  bool stopController(const std::string &controller) {
    std::vector<std::string> controllers;
    controllers.push_back(controller);
    return stopController(controllers);
  }
  bool startController(const std::vector<std::string> &controllers) {
    return switchController(controllers, std::vector<std::string>());
  }
  bool stopController(const std::vector<std::string> &controllers) {
    return switchController(std::vector<std::string>(), controllers);
  }
  bool loadAllControllers() {
    return loadControllers(information_controllers_) && loadControllers(movement_controllers_)
        && loadControllers(calibration_controllers_);
  }
  bool startAllControllers() {
    return startController(information_controllers_) && startController(movement_controllers_)
        && startController(calibration_controllers_);
  }
  bool stopAllControllers() {
    return stopController(information_controllers_) && stopController(movement_controllers_)
        && stopController(calibration_controllers_);
  }

  bool startMovementControllers() { return startController(movement_controllers_); }
  bool stopMovementControllers() { return stopController(movement_controllers_); }
  bool startInformationControllers() { return startController(information_controllers_); }
  bool stopInformationControllers() { return stopController(information_controllers_); }
  bool startCalibrationControllers() { return startController(calibration_controllers_); }
  bool stopCalibrationControllers() { return stopController(calibration_controllers_); }
 private:
  ros::ServiceClient switch_controller_client_;
  ros::ServiceClient load_controllers_client_;
  std::vector<std::string> information_controllers_;
  std::vector<std::string> movement_controllers_;
  std::vector<std::string> calibration_controllers_;
};

}
#endif //RM_FSM_COMMON_CONTROLLER_MANAGER_H_
