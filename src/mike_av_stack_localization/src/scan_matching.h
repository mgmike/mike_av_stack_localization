#include "helper.h"
#include <Eigen/Geometry>
// #include <sensor_msgs/msg/PointCloud2.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/visualization/pcl_visualizer.h>

#ifndef SCANMATCHING_H
#define SCANMATCHING_H


class Scan_Matching : public rclcpp::Node
{
protected:
	PointCloudT::Ptr target;
	bool viz = false;
	pcl::visualization::PCLVisualizer::Ptr viewer;

public:
	Scan_Matching(PointCloudT::Ptr target);
	virtual void set_map(PointCloudT::Ptr target);
	virtual void get_transform(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
	virtual void enable_viz();
	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

};

#endif