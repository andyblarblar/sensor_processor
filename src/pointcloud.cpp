#include <memory>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace message_filters;

class PointCloudMerge : public rclcpp::Node
{
public:
  PointCloudMerge(rclcpp::NodeOptions options) : Node("PointCloud_Concatenate", options)
  {
    // Params
    camera_trans_source = this->declare_parameter("camera_trans_source", "laser_link");
    camera_trans_dest = this->declare_parameter("camera_trans_dest", "base_footprint");

    rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_sensor_data;
    lidar_subscription_.subscribe(this, "/lidar/points", rmw_qos_profile);
    camera_subscription_.subscribe(this, "/camera/points", rmw_qos_profile);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    sync.reset(new Sync(MySyncPolicy(10), lidar_subscription_, camera_subscription_));

    // synchronizer's callback function
    sync->registerCallback(std::bind(&PointCloudMerge::pc_callback, this, std::placeholders::_1, std::placeholders::_2));

    combined_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/combined/points", rclcpp::SensorDataQoS());
  }

private:
  /*
  Function that takes in 2 pointcloud messages
  and concatenates them together then publishes
  it on a single topic
  */
  void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg2)
  {
    if (!camera_trans.has_value())
    {
      camera_trans = tf_buffer->lookupTransform(camera_trans_dest, camera_trans_source, tf2::TimePointZero);
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
      point.z += camera_trans_.transform.translation.z;
    }

    merged = lidar + camera;

    sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(merged, *output);

    output->header.stamp = this->get_clock()->now();
    combined_pc_publisher_->publish(*output);
  }

  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> camera_subscription_;

  typedef sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_publisher_;
  bool merge_point_clouds;

  // Tf stuff
  std::shared_ptr<tf2_ros::TransformListener> transform_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  /// Transform to apply to the camera points
  std::optional<geometry_msgs::msg::TransformStamped> camera_trans{};
  std::string camera_trans_source;
  std::string camera_trans_dest;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  auto sensor_node = std::make_shared<PointCloudMerge>(options);
  exec.add_node(sensor_node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
