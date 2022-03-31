#ifndef SENSOR_PROCESSOR__SENSOR_PROCESSOR_HPP_
#define SENSOR_PROCESSOR__SENSOR_PROCESSOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace SensorProcessor
{
    /// Node that republishes sensor topics to fix their frame_ids. 
    class SensorProcessor : public rclcpp::Node
    {
    public:
        explicit SensorProcessor(rclcpp::NodeOptions options);

    private:
        //image_raw
        void raw_img_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        void raw_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);    
        void raw_laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); 
        void raw_depth_img_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);  
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_raw_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr unfiltered_img_raw_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_depth_img_raw_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr unfiltered_depth_img_raw_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr raw_laser_scan_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr filtered_laser_scan_publisher_;       
    };
}

#endif
