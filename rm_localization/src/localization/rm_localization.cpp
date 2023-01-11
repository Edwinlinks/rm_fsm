//
// Created by luotinkai on 2022/11/3.
//
#pragma once
#include "localization/rm_localization.h"

RmLocalization::RmLocalization() {
  ros::NodeHandle location_nh;
  detetion_sub_ = location_nh.subscribe<apriltag_ros::AprilTagDetectionArray>(
      "tag_detections", 100, &RmLocalization::tagCallBack, this);
  camera_frame_ = getParam(location_nh, "camera_frame",
                           static_cast<std::string>("camera_optical_frame"));
  int moving_average_num = getParam(location_nh, "moving_average_num", 5);
  filter_x_ = new MovingAverageFilter<double>(moving_average_num);
  filter_y_ = new MovingAverageFilter<double>(moving_average_num);
  filter_yaw_x_ = new MovingAverageFilter<double>(moving_average_num);
  filter_yaw_y_ = new MovingAverageFilter<double>(moving_average_num);
}

void RmLocalization::fuseDetectedTags(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  // If no detection
  if (msg->detections.empty())
    return;

  double estimate_x = 0.0;
  double estimate_y = 0.0;
  double estimate_yaw_x = 0.0;
  double estimate_yaw_y = 0.0;
  double weight_sum = 0.0;

  // Lookup Base -> Cam Transform
  tf2::Transform cam2base;
  try {
    geometry_msgs::TransformStamped tf_msg =
        tf_buffer_.lookupTransform("base_link", camera_frame_, ros::Time(0));
    tf2::fromMsg(tf_msg.transform, cam2base);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  // Loop all detections
  for (const auto &detection : msg->detections) {
    unsigned long tag_id;
    tf2::Vector3 cam2tag_v;
    tf2::Quaternion cam2tag_q;

    tag_id = static_cast<unsigned long>(detection.id[0]);
    tf2::fromMsg(detection.pose.pose.pose.position, cam2tag_v);
    tf2::convert(detection.pose.pose.pose.orientation, cam2tag_q);
    tf2::Transform cam2tag;
    cam2tag.setOrigin(cam2tag_v);
    cam2tag.setRotation(cam2tag_q);

    geometry_msgs::TransformStamped camera2tag_msg;
    camera2tag_msg.header.stamp = ros::Time::now();
    camera2tag_msg.header.frame_id =
        "tag_" + std::to_string(static_cast<int>(tag_id));
    camera2tag_msg.child_frame_id = "cam";
    camera2tag_msg.transform = tf2::toMsg(cam2tag.inverse());

    tf_broadcaster_.sendTransform(camera2tag_msg);

    tf2::Transform tag2base;
    tag2base = cam2tag.inverse() * cam2base;

    // Lookup map -> base Transform
    tf2::Transform map2tag;
    try {
      geometry_msgs::TransformStamped tf_msg = tf_buffer_.lookupTransform(
          "map", "tag_" + std::to_string(static_cast<int>(tag_id)),
          ros::Time(0));
      tf2::fromMsg(tf_msg.transform, map2tag);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
    }

    // Base to map
    tf2::Transform map2base;
    map2base = map2tag * tag2base;
    // Send current tag location
    geometry_msgs::TransformStamped map2base_msg;
    map2base_msg.header.stamp = ros::Time::now();
    map2base_msg.header.frame_id = "map";
    map2base_msg.child_frame_id =
        "not_fuse_base" + std::to_string(static_cast<int>(tag_id));
    map2base_msg.transform = tf2::toMsg(map2base);
    tf_broadcaster_.sendTransform(map2base_msg);

    double cur_base_x = map2base.getOrigin().getX();
    double cur_base_y = map2base.getOrigin().getY();
    double cur_base_z = map2base.getOrigin().getZ();
    double cur_base_yaw = yawFromQuat(tf2::toMsg(map2base.getRotation()));

    // Ignore Flying base (>0.2m)
    if (std::abs(cur_base_z) < 0.2) {
      // Calculate Weight
      double cur_weight = 1 / cam2tag_v.length();

      // Sum Up Position Estimate
      estimate_x += cur_weight * cur_base_x;
      estimate_y += cur_weight * cur_base_y;
      estimate_yaw_x += cur_weight * cos(cur_base_yaw);
      estimate_yaw_y += cur_weight * sin(cur_base_yaw);
      weight_sum += cur_weight;
    }
  }

  if (fabs(weight_sum) < 1e-8)
    return;

  // get weighted pose (map -> base)
  tf2::Transform weighted_map2base;

  filter_x_->input(estimate_x / weight_sum);
  filter_y_->input(estimate_y / weight_sum);
  filter_yaw_x_->input(estimate_yaw_y / weight_sum);
  filter_yaw_y_->input(estimate_yaw_x / weight_sum);
  weighted_map2base.setOrigin(
      tf2::Vector3(filter_x_->output(), filter_y_->output(), 0));
  tf2::Quaternion quat_from_yaw;
  quat_from_yaw.setRPY(0.0, 0.0,
                       atan2(filter_yaw_y_->output(), filter_yaw_x_->output()));
  weighted_map2base.setRotation(quat_from_yaw);

  // Send current tag location
  geometry_msgs::TransformStamped new_map2base_msg;
  new_map2base_msg.header.stamp = ros::Time::now();
  new_map2base_msg.header.frame_id = "map";
  new_map2base_msg.child_frame_id = "fused_base";
  new_map2base_msg.transform = tf2::toMsg(weighted_map2base);

  tf_broadcaster_.sendTransform(new_map2base_msg);
  // get base -> odom
  tf2::Transform base2odom;
  try {
    geometry_msgs::TransformStamped tf_msg =
        tf_buffer_.lookupTransform("base_link", "odom", ros::Time(0));
    tf2::fromMsg(tf_msg.transform, base2odom);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  // set map -> odom
  map2odom_ = weighted_map2base * base2odom;

  geometry_msgs::TransformStamped map2odom_msg;
  map2odom_msg.header.stamp = ros::Time::now();
  map2odom_msg.header.frame_id = "map";
  map2odom_msg.child_frame_id = "odom";
  map2odom_msg.transform = tf2::toMsg(map2odom_);
  tf_broadcaster_.sendTransform(map2odom_msg);
}

void RmLocalization::tagCallBack(
    const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
  fuseDetectedTags(msg);
}