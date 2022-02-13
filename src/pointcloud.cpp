// MIT License
//
// Copyright (c) Intelligent Systems Club
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>


using namespace message_filters;

class PointCloudMerge : public rclcpp::Node
{
public: 
  PointCloudMerge(rclcpp::NodeOptions options): Node("PointCloud_Concatenate", options)
  {
  
  lidar_subscription_.subscribe(this, "/lidar/points");
  camera_subscription_.subscribe(this, "/camera/points");

  sync.reset(new Sync(MySyncPolicy(10), lidar_subscription_, camera_subscription_)); 
  
  // synchronizer's callback function 
  sync -> registerCallback(std::bind(&PointCloudMerge::pc_callback, this, std::placeholders::_1, std::placeholders::_2));  
  
  combined_pc_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/combined/points", rclcpp::SensorDataQoS());  
  }
  
private:

  /*
  Function that takes in 2 pointcloud messages 
  and concatenates them together then publishes
  it on a single topic
  */
  void pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg1, const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg2) 
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
  
  typedef sync_policies::ApproximateTime<sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2> MySyncPolicy;
  typedef Synchronizer<MySyncPolicy> Sync;
  std::shared_ptr<Sync> sync;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr combined_pc_publisher_; 
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
