
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>

#include <stdlib.h>
#include <stdint.h>

#include <omp.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

#include "SLAM/Localizer.h"
#include "SLAM/Mapper.h"
#include "SLAM/ICP.h"
#include "SLAM/GraphMap.h"

ros::Subscriber pointcloud_subscriber_;
ros::Subscriber gyroscope_subscriber;
ros::Subscriber accelerometer_subscriber;

std::deque<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>> pointclouds;
std::deque<std::pair<ros::Time, sensor_msgs::Imu>> gyroscope_samples;
std::deque<std::pair<ros::Time, sensor_msgs::Imu>> accelerometer_samples;

std::mutex producer_mutex_;
std::condition_variable producer_notification;
std::mutex consumer_mutex_;
std::condition_variable consumer_notification;
bool product_valid = false;
bool production_possible = true;

void testCallback(const sensor_msgs::Joy::ConstPtr& data){
	product_valid = true;
	consumer_notification.notify_one();
}

void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg_pointcloud){
		
	if(!production_possible){
		std::unique_lock<std::mutex> lock(producer_mutex_);
		producer_notification.wait(lock, []{return production_possible;});
	}
	
	// Containers
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PointCloud<pcl::PointXYZ>::Ptr standard_cloud(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	// Convert to PCL data type
	pcl_conversions::toPCL(*msg_pointcloud, *cloud); // "moveFromPCL" could be used if msg_pointcloud wasn't "const"

	pcl::fromPCLPointCloud2(*cloud, *standard_cloud);

	// Perform the actual filtering
	//~ pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	//~ sor.setInputCloud (cloudPtr);
	//~ sor.setLeafSize (0.1, 0.1, 0.1);
	//~ sor.filter (*cloud_filtered);
	
	pointclouds.push_back(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>(msg_pointcloud->header.stamp, standard_cloud));
	product_valid = true;
    consumer_notification.notify_one();
}

void gyroscopeCallback(const sensor_msgs::Imu::ConstPtr & gyroData){
	sensor_msgs::Imu manipulatable_data = *gyroData;
	gyroscope_samples.push_front(std::pair<ros::Time, sensor_msgs::Imu>(gyroData->header.stamp, manipulatable_data));
}

void accelerometerCallback(const sensor_msgs::Imu::ConstPtr & accelData){
	sensor_msgs::Imu manipulatable_data = *accelData;
	accelerometer_samples.push_front(std::pair<ros::Time, sensor_msgs::Imu>(accelData->header.stamp, manipulatable_data));
}

void shutdownThread(){
	ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
	}
	product_valid = true;
	consumer_notification.notify_one();
}

int main(int argc, char** argv){
	ROS_INFO("Starting the point_cloud_reader_node");
	ros::init(argc, argv, "point_cloud_reader_node");
	ros::NodeHandle nh;
	ros::NodeHandle n_priv("~");
	
	std::string pointcloud_topic = "";
	
	n_priv.getParam("pointcloud_topic", pointcloud_topic);
	
	ROS_INFO("Got Param: %s", pointcloud_topic.c_str());
	
	//Subscriber starten
	pointcloud_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 10, pointCloudCallback);
	//~ pointcloud_subscriber_ = nh.subscribe<sensor_msgs::Joy>("/joy", 1, testCallback);
	gyroscope_subscriber = nh.subscribe<sensor_msgs::Imu>("gyro/sample", 100, gyroscopeCallback);
	accelerometer_subscriber = nh.subscribe<sensor_msgs::Imu>("accel/sample", 100, accelerometerCallback);
	
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	std::thread shutdown_security(shutdownThread);
	
	std::shared_ptr<localizer::Localizer> localizer_ptr;
	std::shared_ptr<mapper::Mapper> mapper_ptr;
	
	// TODO: Localizer und Mapper in dynamic Reconfigure Funktion austauschbar machen und per Parameter initialisieren
	
	localizer_ptr.reset(new localizer::ICP());
	mapper_ptr.reset(new mapper::GraphMap());
	
	bool firstFrame = true;
	
	while(ros::ok())
	{
		ROS_INFO("Number of unprocessed pointclouds: %lu", pointclouds.size());
		if(product_valid)
		{
			if(pointclouds.size() > 0)
			{
				production_possible = false;
				std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud = pointclouds.front();
				pointclouds.pop_front();
				production_possible = true;
				producer_notification.notify_all();
				if(firstFrame){
					//~ firstFrame = false;
					
					//~ for(pcl::PointCloud<pcl::PointXYZ>::iterator iter = current_pointcloud.second->begin(); iter != current_pointcloud.second->end(); ++iter)
					//~ {
						//~ std::cout << "(" << (*iter).x << "," << (*iter).y << "," << (*iter).z << ")" << std::endl;
					//~ }
					
					//~ ROS_INFO("Working off Pointcloud of size: %u x %u",current_pointcloud.second->width, current_pointcloud.second->height);
					mapper_ptr->add_cloud(current_pointcloud.second);
				}
			}
			if(pointclouds.size() == 0)
			{
				product_valid = false;
			}
		}
		else
		{
			std::unique_lock<std::mutex> lock(consumer_mutex_);
			consumer_notification.wait(lock, []{return product_valid;});
		}
	}
}
