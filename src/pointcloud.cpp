#include <memory>
#include <string>
#include <cstring>

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
	
	rmw_qos_profile_t rmw_qos_profile = rmw_qos_profile_sensor_data;
	lidar_subscription_.subscribe(this, "/kohm/filtered_points", rmw_qos_profile);
	camera_subscription_.subscribe(this, "/kohm/camera_points", rmw_qos_profile);

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
