//
// Created by luotinkai on 2022/11/9.
//
#include <rm_common/ros_utilities.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tags_distribute");
  ros::NodeHandle nh_private("~");

  tf2_ros::StaticTransformBroadcaster br;
  XmlRpc::XmlRpcValue tags_list;
  nh_private.getParam("tags_list", tags_list);

  for (int i = 0; i < tags_list.size(); ++i) {
    ROS_ASSERT(tags_list[i].getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(tags_list[i][0].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
               tags_list[i][1].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
               tags_list[i][2].getType() == XmlRpc::XmlRpcValue::TypeDouble &&
               tags_list[i][3].getType() == XmlRpc::XmlRpcValue::TypeInt);

    tf2::Transform map2tag;
    map2tag.setOrigin(tf2::Vector3(static_cast<double>(tags_list[i][0]),
                                   static_cast<double>(tags_list[i][1]), 0.0));
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, static_cast<double>(tags_list[i][2]));
    map2tag.setRotation(quaternion);

    geometry_msgs::TransformStamped trans_stamped;
    trans_stamped.header.stamp = ros::Time::now();
    trans_stamped.header.frame_id = "map";
    trans_stamped.child_frame_id =
        "tag" + std::to_string(static_cast<int>(tags_list[i][3]));
    ROS_INFO_STREAM("Sent transform from map to tag" +
                    std::to_string(static_cast<int>(tags_list[i][3])));
    trans_stamped.transform = tf2::toMsg(map2tag);

    br.sendTransform(trans_stamped);
  }

  ros::spin();
  return 0;
}