//
// Created by luotinkai on 2022/11/9.
//
#include <tf/tf.h>
#include <tf2_ros/static_transform_broadcaster.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "tags_lut");

  ros::NodeHandle nh_private("~");

  tf2_ros::StaticTransformBroadcaster br;
  int x_tags, y_tags;
  double dis_tags;
  nh_private.param<std::int32_t>("x_tags", x_tags, 1);
  nh_private.param<std::int32_t>("y_tags", y_tags, 1);
  nh_private.param<std::double_t>("dis_tags", dis_tags, 0.8);

  //  ros::Rate rate(10);
  //  while (ros::ok()) {
  geometry_msgs::TransformStamped tf_map_tag;
  tf_map_tag.header.frame_id = "map";
  tf_map_tag.header.stamp = ros::Time::now();
  // Init Static Transforms
  int x = 0;
  int y = 329;
  int tag_id = x * y_tags + y + 1;
  double tag_x = dis_tags * x;
  double tag_y = -dis_tags * 1.0;

  tf_map_tag.child_frame_id = "tag_" + std::to_string(tag_id);
  tf_map_tag.transform.translation.x = tag_x;
  tf_map_tag.transform.translation.y = tag_y;
  tf_map_tag.transform.translation.z = 0.0;

  tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0, 0, 0),
                        tf_map_tag.transform.rotation);

  br.sendTransform(tf_map_tag);
  //    rate.sleep();
  //  }
  ros::spin();
  return 0;
}