#ifndef SENSOR_PROCESSOR__SENSOR_PROCESSOR_HPP_
#define SENSOR_PROCESSOR__SENSOR_PROCESSOR_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"
#include <nav_msgs/msg/odometry.hpp>

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
        void raw_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);       
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_raw_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr unfiltered_img_raw_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_imu_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
        
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr raw_odom_subscription_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_raw_publisher_;        
    };
}

#endif
