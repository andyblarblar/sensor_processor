
#include "pointcloud_merge/pointcloud.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

PointCloudMerge::PointCloudMerge(rclcpp::NodeOptions options)
: Node("PointCloud_Concatenate", options)
{
  // Params
  camera_trans_source = this->declare_parameter("camera_trans_source", "base_footprint");
  camera_trans_dest = this->declare_parameter("camera_trans_dest", "laser_link");
  tf_timeout = this->declare_parameter("tf_timeout", 0.03);
    
  // Tf2
  tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);  
       
  // message filter synchronizer and publisher
  rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_sensor_data;
  lidar_subscription_.subscribe(this, "/lidar/points", rmw_qos_profile);
  camera_subscription_.subscribe(this, "/camera/points", rmw_qos_profile);
  sync.reset(new Sync(MySyncPolicy(10), lidar_subscription_, camera_subscription_));

  // synchronizer's callback function
  sync->registerCallback(std::bind(&PointCloudMerge::pc_callback, this, std::placeholders::_1, std::placeholders::_2));

  combined_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/combined/points", rclcpp::SensorDataQoS());
}

/*
Function that takes in 2 pointcloud messages
and concatenates them together then publishes
it on a single topic
*/
void PointCloudMerge::pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2)
{
  if (!camera_trans.has_value())
  {
    camera_trans = tf_buffer->lookupTransform(camera_trans_dest, camera_trans_source, this->now(),
      rclcpp::Duration::from_seconds(tf_timeout));
  }

  auto &camera_trans_ = camera_trans.value();

  pcl::PointCloud<pcl::PointXYZ> lidar, camera, merged;

  pcl::fromROSMsg(*msg1, lidar);
  pcl::fromROSMsg(*msg2, camera);

  // Translate all points from the camera to move it into the dest frame.
  // This is useful to move the camera into the laser_link frame, which is
  // what the concatinated point cloud is published to.
  for (auto &point : camera)
  {
    point.x += camera_trans_.transform.translation.x;
    point.y += camera_trans_.transform.translation.y;
    // Doing the z transform causes pcl-ls to fail, so we don't do it.
  }

  merged = lidar + camera;

  sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
  pcl::toROSMsg(merged, *output);

  output->header.stamp = this->get_clock()->now();
  combined_pc_publisher_->publish(*output);
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto cloud_node = std::make_shared<PointCloudMerge>(options);
  exec.add_node(cloud_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
