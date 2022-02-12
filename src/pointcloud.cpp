#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>


class PointCloudMerge : public rclcpp::Node
{
public:
  PointCloudMerge(): Node("PointCloud Concatenate")
  {
    lidar_subscription_.subscribe(this, "/kohm/filtered_points");
    camera_subscription_.subscribe(this, "/kohm/camera_points");

    sync_ = std::make_shared<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, 
      sensor_msgs::msg::PointCloud2>>(lidar_subscription_, camera_subscription_, 10);
      
    sync_->registerCallback(std::bind(&PointCloudMerge::pc_callback, this, std::placeholders::_1, std::placeholders::_2));
    combined_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/combined/points", rclcpp::SensorDataQoS());  
  }

private:
  void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2) const
  {
    pcl::PointCloud<pcl::PointXYZ> lidar, camera, merged;

    pcl::fromROSMsg(*msg1, lidar);
    pcl::fromROSMsg(*msg2, camera);
    
    merged = lidar + camera;
    
    sensor_msgs::msg::PointCloud2::SharedPtr output(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(merged, *output);
    combined_pc_publisher_->publish(*output);

  }
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> lidar_subscription_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> camera_subscription_;
  std::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>> sync_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_publisher_; 
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudMerge>());
  rclcpp::shutdown();

  return 0;
}
