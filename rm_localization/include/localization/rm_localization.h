//
// Created by luotinkai on 2022/11/3.
//

#pragma

#include <apriltag_ros/AprilTagDetectionArray.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <string>
#include <tf/tf.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <rm_common/filters/filters.h>
#include <rm_common/ori_tool.h>

class RmLocalization {
public:
  RmLocalization();
  void
  fuseDetectedTags(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
  void tagCallBack(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

private:
  tf2::Transform map2odom_;
  std::string camera_frame_;

  MovingAverageFilter<double> *filter_x_, *filter_y_, *filter_yaw_x_,
      *filter_yaw_y_;

  ros::Subscriber tag_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};