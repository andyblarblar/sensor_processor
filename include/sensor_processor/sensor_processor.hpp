#ifndef SENSOR_PROCESSOR__SENSOR_PROCESSOR_HPP_
#define SENSOR_PROCESSOR__SENSOR_PROCESSOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.h"
#include "std_msgs/msg/header.hpp"
#include <memory>

namespace SensorProcessor
{
    class SensorProcessor : public rclcpp::Node
    {
    public:
        explicit SensorProcessor(rclcpp::NodeOptions options);

    private:
        //image_raw
        void raw_img_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_img_raw_subscription_;
        rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr unfiltered_img_raw_publisher_;

        
    };
}

#endif
