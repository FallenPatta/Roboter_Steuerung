#ifndef POINTCLOUDDISPLAY_H_
#define POINTCLOUDDISPLAY_H_

#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>
#include <random>
#include <ctime>

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
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Point.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

class PointCloudDisplay{
	public:
		PointCloudDisplay(ros::Publisher * p1_, ros::Publisher * p2_, ros::Publisher * p3_):
		p1(p1_), p2(p2_), p3(p3_)
		{
		}
		
	void publishCloud1(pcl::PointCloud<pcl::PointXYZ> & to_send){
		sensor_msgs::PointCloud2 msg_pointcloud;
		pcl::PCLPointCloud2 cloud;
		pcl::toPCLPointCloud2(to_send, cloud);
		pcl_conversions::fromPCL(cloud, msg_pointcloud);
		msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
		p1->publish(msg_pointcloud);
	}

	void publishCloud2(pcl::PointCloud<pcl::PointXYZ> & to_send){
		sensor_msgs::PointCloud2 msg_pointcloud;
		pcl::PCLPointCloud2 cloud;
		pcl::toPCLPointCloud2(to_send, cloud);
		pcl_conversions::fromPCL(cloud, msg_pointcloud);
		msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
		p2->publish(msg_pointcloud);
	}

	void publishCloud3(pcl::PointCloud<pcl::PointXYZ> & to_send){
		sensor_msgs::PointCloud2 msg_pointcloud;
		pcl::toROSMsg(to_send, msg_pointcloud);
		msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
		p3->publish(msg_pointcloud);
	}
	
	void display(cv::Mat& image1, cv::Mat& image2, cv::Mat& descriptors1, cv::Mat& descriptors2, std::vector<cv::KeyPoint>& keypoints1, std::vector<cv::KeyPoint>& keypoints2){
		cv::FlannBasedMatcher matcher;
		std::vector<cv::DMatch> matches;
		matcher.match(descriptors1, descriptors2, matches);
		
		double mindist = 10000;

		for(cv::DMatch match : matches)
		{
			if(match.distance < mindist)
				mindist = match.distance;
		}
			
		std::vector<cv::DMatch> good_matches;
		
		for(cv::DMatch match : matches)
		{
			if(match.distance <= std::max(2*mindist, 0.05))
				good_matches.push_back(match);
		}
		
		cv::Mat img_matches;
		drawMatches(image1, keypoints1, image2, keypoints2, good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
		cv::imshow( "Good Matches", img_matches);
		cv::waitKey(25);
	}
		
	private:
	
	ros::Publisher * p1;
	ros::Publisher * p2;
	ros::Publisher * p3;
	
	
};

#endif //POINTCLOUDDISPLAY_H_
