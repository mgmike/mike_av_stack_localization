#include "rclcpp/rclcpp.hpp"
#include <thread>
#include <string>
#include <unordered_map>

#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl_conversions/pcl_conversions.h>


#include "helper.h"
#include "scan_matching.h"
#include "ndt.h"
#include "icp.h"
#include "icps.h"

rclcpp::Node::SharedPtr node = NULL;
// rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

void callback(const sensor_msgs::msg::PointCloud2& cloud_msg){
    if (node != NULL){
        RCLCPP_INFO(node->get_logger(), "In Callback");
    }
    // Create pcl point cloud
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloudfiltered;

}


int main(int argc, char** argv){
    // ros::init(argc, argv, "localization");
    // ros::NodeHandle nh("~");
    rclcpp::init(argc, argv);
    node = rclcpp::Node::make_shared("localization");
    RCLCPP_INFO(node->get_logger(), "Hello world!");

    Scan_Matching* scan_matching;

    // I added the ability to change values through input params for rapid testing
	string map_name = "map.pcd";
    string map_directory = "/media/mike/Storage/Documents/autonomous_sim/src/mike_av_stack/scripts/localization/maps/";
	int iters = 10;
	int dist = 2;
  	int cp_size = 5000;
  	double leafSize = 0.5;
	bool need_to_write = true;
    // std::string topic("/carla/ego_vehicle/lidar/lidar1/point_cloud");
    std::string topic("/carla/ego_vehicle/lidar/lidar1/point_cloud_full");

    std::string param;
    int param_int;
    // If the parameter 'map_name' exists and it is type string, then return true
    if (node->get_parameter("iters", param_int)){
        iters = param_int;
    }
    if (node->get_parameter("map_name", param)){
        map_name = param; 
    }    
    if (node->get_parameter("topic", param)){
        topic = param;
    }

    // Initialize visualization
    // pcl::visualization::PCLVisualizer::Ptr viewer;
    // viewer.reset(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  	// viewer->setBackgroundColor (0, 0, 0);

    // Load map
	PointCloudT::Ptr mapCloud(new PointCloudT);
  	pcl::io::loadPCDFile(map_directory + map_name, *mapCloud);
  	RCLCPP_INFO_STREAM(node->get_logger(), "Loaded " << mapCloud->points.size() << " data points from " << map_name);
	// Flip the points. For some reason, they are flipped
	PointCloudT::Ptr flipped(new PointCloudT);
	for (auto point : mapCloud->points){
		flipped->points.push_back(PointT(point.x, -1.0 * point.y, point.z));
	}
	mapCloud = flipped;
	// renderPointCloud(viewer, mapCloud, "map", Color(0,0,1));

    // Get gps position
	Pose pose(Point(65.516594,7.808423,0.275307), Rotate(0.855823,0.0,0.0));

    // Assign the type of scan matching algorithm to scan_matching.
    if (node->get_parameter("scan_matching_algorithm", param)){
        if (param == "ndt"){
		    scan_matching = new NDT(mapCloud, pose, iters);
        } else if (param == "icp"){
		    scan_matching = new ICP(mapCloud, pose, iters);
        } else if (param == "icps"){
		    scan_matching = new ICPS(mapCloud, pose, iters, dist);
        } else {return 0;}
        //Move inside later
        RCLCPP_INFO(node->get_logger(), "Setting up subscriber");
    }

    bool viz;
    if (node->get_parameter("viz", viz)){
        if (viz){
            scan_matching->enable_viz();
        }
    }


    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}