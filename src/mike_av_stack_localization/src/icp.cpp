#include "icp.h"

std::string gnss_topic = "/carla/ego_vehicle/gnss/gnss1/fix";
std::string imu_topic = "/carla/ego_vehicle/imu/imu1";

ICP::ICP(PointCloudT::Ptr t, Pose sp, int iter): Scan_Matching(t), pose(sp), iterations(iter) {
	RCLCPP_INFO(this->get_logger(), "In ICP!");
	size_t msgQueueSize = 10;
	rclcpp::QoS qos_profile(msgQueueSize);
	qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE); // Ensure reliable delivery
    qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    initTransform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
    gnss_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		gnss_topic, qos_profile, std::bind(&ICP::gnss_update, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
		imu_topic, qos_profile, std::bind(&ICP::imu_update, this, std::placeholders::_1));
	
}

void ICP::gnss_update(const sensor_msgs::msg::NavSatFix::SharedPtr gnss){
	RCLCPP_INFO(this->get_logger(), "Got gnss!");
	if (!gnss_rdy) {
		gnss_rdy = true;
		ps_mutex.lock();
		pose.position.x = gnss->longitude;
		pose.position.y = gnss->latitude;
		pose.position.z = gnss->altitude;
		ps_mutex.unlock();
	}
}

void ICP::imu_update(const sensor_msgs::msg::Imu::SharedPtr imu){
	// imu->orientation is a geometry_msgs/Quaternion, and must be converted to euiler for pose 
	RCLCPP_INFO(this->get_logger(), "Got imu!");
	if (!imu_rdy) {
		imu_rdy = true;
		Rotate r;
		tf2::Quaternion qtf;
		tf2::fromMsg(imu->orientation, qtf);
		getEuiler(qtf, r);
		rt_mutex.lock();
		pose.rotation = r;
		rt_mutex.unlock();
	}
}

void ICP::get_transform(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg){
	RCLCPP_INFO(this->get_logger(), "Got point cloud!");

    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

	// Create pcl point cloud
	pcl::PCLPointCloud2 cloud = pcl::PCLPointCloud2();
	// pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	// Convert to pcl
	pcl_conversions::toPCL(*cloud_msg, cloud);
	PointCloudT::Ptr source(new PointCloudT);
	pcl::fromPCLPointCloud2(cloud, *source);

	// Make voxel grid
	PointCloudT::Ptr filteredSource(new PointCloudT);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(source);
	vg.setLeafSize(leafSize, leafSize, leafSize);
	vg.filter(*filteredSource);
 
	// 1. Transform the source to the pose
	it_mutex.lock();
	ps_mutex.lock();
	rt_mutex.lock();
	initTransform = transform3D(pose.rotation.yaw, pose.rotation.pitch, pose.rotation.roll, pose.position.x, pose.position.y, pose.position.z);
	rt_mutex.unlock();
	ps_mutex.unlock();
	it_mutex.unlock();
	PointCloudT::Ptr transformSource(new PointCloudT);
  	pcl::transformPointCloud(*filteredSource, *transformSource, initTransform);
  
	//2. Create the PCL icp object
	pcl::console::TicToc time;
  	time.tic ();
  	pcl::IterativeClosestPoint<PointT, PointT> icp;

	//3. Set the icp object's values
  	icp.setMaximumIterations(iterations);
  	icp.setInputSource(transformSource);
  	icp.setInputTarget(target);
  	icp.setMaxCorrespondenceDistance(2);
  
	//4. Call align on the icp object
  	PointCloudT::Ptr tempSource (new PointCloudT);
  	icp.align(*tempSource);
  
  	if(icp.hasConverged()){
  		RCLCPP_INFO(this->get_logger(), "ICP has converged");
		tm_mutex.lock();
		transformation_matrix = icp.getFinalTransformation().cast<double>();
		it_mutex.lock();
		transformation_matrix = transformation_matrix * initTransform;
		it_mutex.unlock();
		ps_mutex.lock();
		rt_mutex.lock();
		pose = getPose(transformation_matrix);
		rt_mutex.unlock();
		ps_mutex.unlock();
		tm_mutex.unlock();
	
    }

	if (viz){
		// Transform scan so it aligns with ego's actual pose and render that scan
		PointCloudT::Ptr transformed_scan (new PointCloudT);
		pcl::transformPointCloud(*filteredSource, *transformed_scan, transformation_matrix);
		viewer->removePointCloud("scan");
		renderPointCloud(viewer, transformed_scan, "scan", Color(1,0,0) );

		Pose estimatedPose = getPose(transformation_matrix);

		viewer->removeAllShapes();
		drawCar(viewer,  Color(0,1,0), estimatedPose, 1, 0.35);

		viewer->spinOnce();
	}
}