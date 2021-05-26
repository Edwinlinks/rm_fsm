//
// Created by astro on 2021/5/15.
//

#include "rm_fsm/common/controller_manager.h"
namespace rm_fsm {

ControllerManager::ControllerManager(ros::NodeHandle &nh) {
  switch_controller_client_ =
      nh.serviceClient<controller_manager_msgs::SwitchController>("/controller_manager/switch_controller");
  load_controllers_client_ =
      nh.serviceClient<controller_manager_msgs::LoadController>("/controller_manager/load_controller");
  XmlRpc::XmlRpcValue controllers;
  if (!nh.getParam("information_controllers", controllers))
    ROS_INFO("No information controllers defined");
  ROS_ASSERT(controllers.getType() == XmlRpc::XmlRpcValue::TypeArray);
  for (int i = 0; i < controllers.size(); ++i)
    information_controllers_.push_back(controllers[i]);
  if (!nh.getParam("movement_controllers", controllers))
    ROS_INFO("No movement controllers defined");
  for (int i = 0; i < controllers.size(); ++i)
    movement_controllers_.push_back(controllers[i]);
  if (!nh.getParam("calibration_controllers", controllers))
    ROS_INFO("No calibration controllers defined");
  for (int i = 0; i < controllers.size(); ++i)
    calibration_controllers_.push_back(controllers[i]);
}

bool ControllerManager::loadControllers(const std::vector<std::string> &controllers) {
  load_controllers_client_.waitForExistence();
  controller_manager_msgs::LoadController load_controller;
  bool is_success = true;
  for (auto &controller : controllers) {
    load_controller.request.name = controller;
    load_controllers_client_.call(load_controller);
    if (load_controller.response.ok)
      ROS_INFO("loaded %s", controller.c_str());
    else {
      ROS_ERROR("fail to load %s", controller.c_str());
      is_success = false;
    }
  }
  return is_success;
}

bool ControllerManager::switchController(const std::vector<std::string> &start, const std::vector<std::string> &stop) {
  controller_manager_msgs::SwitchController switch_controller;
  switch_controller.request.strictness = switch_controller.request.BEST_EFFORT;
  switch_controller.request.start_asap = true;
  switch_controller.request.timeout = 0.1;
  for (auto &controller : start)
    switch_controller.request.start_controllers.push_back(controller);
  for (auto &controller : stop)
    switch_controller.request.stop_controllers.push_back(controller);
  return switch_controller_client_.call(switch_controller);
}

}