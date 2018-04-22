
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
#include <tf/transform_datatypes.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Geometry>

#include "SLAM/FeatureGenerator.h"
#include "SLAM/Localizer.h"
#include "SLAM/Mapper.h"
#include "SLAM/ICP.h"
#include "SLAM/GraphMap.h"
#include <SLAM/SLAMNode.h>
#include <SLAM/SLAMGraph.h>

struct PointXYZ{
	float x,y,z,u;
};

int point_transform(PointXYZ* vector_array, const int array_size, float* mat_4x4);
int testmain(float* vector_array, float* result_array, const int array_size, float* mat_4x4);

ros::Subscriber pointcloud_subscriber_;
ros::Subscriber color_image_subscriber;

ros::Publisher pointcloud_publisher_;
ros::Publisher pointcloud_publisher_2_;
ros::Publisher pointcloud_publisher_3_;

std::deque<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>> pointclouds;

std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>> transformed_pointclouds;

std::pair<ros::Time, std::shared_ptr<cv::Mat>> current_color_frame_;
int frameNum = 0;

//~ std::shared_ptr<FeatureGenerator> color_feature_generator;
//~ cv::Ptr<cv::AKAZE> color_feature_generator;
cv::Ptr<cv::Feature2D> color_feature_generator;
const double akaze_color_thresh = 1e-3;

std::mutex producer_mutex_;
std::condition_variable producer_notification;
std::mutex consumer_mutex_;
std::condition_variable consumer_notification;
bool product_valid = false;
bool production_possible = true;

std::mutex color_image_mutex;

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
	
	pointclouds.push_back(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>(msg_pointcloud->header.stamp, standard_cloud));
	product_valid = true;
    consumer_notification.notify_one();
}

void publishCloud1(pcl::PointCloud<pcl::PointXYZ> & to_send){
	sensor_msgs::PointCloud2 msg_pointcloud;
	pcl::PCLPointCloud2 cloud;
	pcl::toPCLPointCloud2(to_send, cloud);
	pcl_conversions::fromPCL(cloud, msg_pointcloud);
	msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
	pointcloud_publisher_.publish(msg_pointcloud);
}

void publishCloud2(pcl::PointCloud<pcl::PointXYZ> & to_send){
	sensor_msgs::PointCloud2 msg_pointcloud;
	pcl::PCLPointCloud2 cloud;
	pcl::toPCLPointCloud2(to_send, cloud);
	pcl_conversions::fromPCL(cloud, msg_pointcloud);
	msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
	pointcloud_publisher_2_.publish(msg_pointcloud);
}

void publishCloud3(pcl::PointCloud<pcl::PointXYZ> & to_send){
	sensor_msgs::PointCloud2 msg_pointcloud;
	pcl::toROSMsg(to_send, msg_pointcloud);
	msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
	pointcloud_publisher_3_.publish(msg_pointcloud);
}

void colorImageCallback(const sensor_msgs::Image::ConstPtr & color_image){
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(color_image, sensor_msgs::image_encodings::TYPE_8UC3);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
	if(cv_ptr->image.rows <= 0 || cv_ptr->image.cols <=0)
	{
		return;
	}
	
	// Update GUI Window
	cv::MatIterator_<cv::Vec3b> it, end;
	for(it = cv_ptr->image.begin<cv::Vec3b>(), end = cv_ptr->image.end<cv::Vec3b>(); it != end; ++it)
	{
		*it = cv::Vec3b((*it)[2], (*it)[1], (*it)[0]);
	}
    
	{
		std::unique_lock<std::mutex> lock_for_scope(color_image_mutex);
		current_color_frame_ = std::pair<ros::Time, std::shared_ptr<cv::Mat>>(ros::Time::now(), std::make_shared<cv::Mat>(cv_ptr->image));
		frameNum++;
	}
}

void shutdownThread(){
	ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
	}
	product_valid = true;
	consumer_notification.notify_one();
}

void sparse_ification(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, int constant){
	int num = 0;
	for(pcl::PointXYZ point : *input)
	{
		num++;
		if(num % constant == 0)
		{
			if(!(point.x == 0 && point.y == 0 && point.z == 0))
				output->push_back(point);
			num += ( rand() % constant - (constant/2) );
		}
	}
}

void sparse_ification(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, int constant){
	int num = 0;
	for(pcl::PointXYZ point : *input)
	{
		num++;
		if(num % constant == 0)
		{
			if(!(point.x == 0 && point.y == 0 && point.z == 0))
				output->push_back(point);
			num += ( rand() % constant - (constant/2) );
		}
	}
}

void transform_cloud_by_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix){
	float matrix[16] = {transform_matrix(0,0), transform_matrix(0,1), transform_matrix(0,2), transform_matrix(0,3), 
						transform_matrix(1,0), transform_matrix(1,1), transform_matrix(1,2), transform_matrix(1,3), 
						transform_matrix(2,0), transform_matrix(2,1), transform_matrix(2,2), transform_matrix(2,3), 
						transform_matrix(3,0), transform_matrix(3,1), transform_matrix(3,2), transform_matrix(3,3)};
	
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> * points = &cloud->points;
	PointXYZ * floated_points = (PointXYZ*)&((*points)[0]);
	point_transform(floated_points, points->size(), matrix);
}

void match_and_display(cv::Mat image1, cv::Mat image2){
	std::unique_lock<std::mutex> lock_for_scope1(color_image_mutex);
	std::vector<cv::KeyPoint> keypoints1;
	cv::Mat descriptors1;
	color_feature_generator->detectAndCompute(image1, cv::noArray(), keypoints1, descriptors1);
	
	if(keypoints1.size() < 2)
		return;
	
	std::vector<cv::KeyPoint> keypoints2;
	cv::Mat descriptors2;
	color_feature_generator->detectAndCompute(image2, cv::noArray(), keypoints2, descriptors2);
	
	if(keypoints2.size() < 2)
		return;
	
	//~ double keypoint_vector1[2];
	//~ double keypoint_unit1 = 0;
	//~ keypoint_vector1[0] = 0;
	//~ keypoint_vector1[1] = 0;
	//~ for(cv::KeyPoint k : keypoints1)
	//~ {
		//~ keypoint_vector1[0] += cos(k.angle);
		//~ keypoint_vector1[1] += sin(k.angle);
	//~ }
	//~ keypoint_unit1 = sqrt(pow(keypoint_vector1[0],2) + pow(keypoint_vector1[1],2));
	
	//~ double keypoint_vector2[2];
	//~ double keypoint_unit2 = 0;
	//~ keypoint_vector2[0] = 0;
	//~ keypoint_vector2[1] = 0;
	//~ for(cv::KeyPoint k : keypoints2)
	//~ {
		//~ keypoint_vector2[0] += cos(k.angle);
		//~ keypoint_vector2[1] += sin(k.angle);
	//~ }
	//~ keypoint_unit2 = sqrt(pow(keypoint_vector2[0],2) + pow(keypoint_vector2[1],2));
	
	//~ std::cout << "HyperKeypoint Difference: " << keypoint_unit1 - keypoint_unit2 << std::endl;
					
	if(descriptors1.type() != CV_32F)
	{
		descriptors1.convertTo(descriptors1, CV_32F);
	}
	
	if(descriptors2.type() != CV_32F)
	{
		descriptors2.convertTo(descriptors2, CV_32F);
	}
	
	if(descriptors1.rows == 0)
		return;
	if(descriptors2.rows == 0)
		return;
					
	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(descriptors1, descriptors2, matches);
	
	double mindist = 10000;
	
	for(cv::DMatch match : matches)
	{
		if(match.distance < mindist)
			mindist = match.distance;
	}
	
	if(mindist >= 100){
		cv::waitKey(1);
		return;
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
	cv::waitKey(1);
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
	pointcloud_subscriber_ = nh.subscribe<sensor_msgs::PointCloud2>(pointcloud_topic, 1, pointCloudCallback);
	color_image_subscriber = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, colorImageCallback);
	
	pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1);
	pointcloud_publisher_2_ = nh.advertise<sensor_msgs::PointCloud2>("/reference_debug_cloud", 1);
	pointcloud_publisher_3_ = nh.advertise<sensor_msgs::PointCloud2>("/registered_debug_cloud", 1);
	
	//~ cv::Ptr<cv::AKAZE> akaze_generator = cv::AKAZE::create();
	//~ akaze_generator->setThreshold(akaze_color_thresh);
	//~ color_feature_generator = akaze_generator;
	color_feature_generator = cv::ORB::create();
	
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_matcher;
	//~ pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp_matcher;
	icp_matcher.setTransformationEpsilon (0);
	icp_matcher.setMaxCorrespondenceDistance (0.05);
	icp_matcher.setMaximumIterations(5);
	icp_matcher.setEuclideanFitnessEpsilon (0);
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	std::thread shutdown_security(shutdownThread);
	
	std::shared_ptr<localizer::Localizer> localizer_ptr;
	std::shared_ptr<mapper::Mapper> mapper_ptr;
	
	// TODO: Localizer und Mapper in dynamic Reconfigure Funktion austauschbar machen und per Parameter initialisieren
	
	localizer_ptr.reset(new localizer::ICP());
	mapper_ptr.reset(new mapper::GraphMap());
	
	cv::namedWindow("Good Matches");	
	
	bool firstFrame = true;
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pointcloud;
	pcl::PointCloud<pcl::PointXYZ> map_pointcloud;
	Eigen::Matrix4f pose_matrix;
	pose_matrix(0,0) = 1; pose_matrix(1,0) = 0; pose_matrix(2,0) = 0; pose_matrix(3,0) = 0;
	pose_matrix(0,1) = 0; pose_matrix(1,1) = 1; pose_matrix(2,1) = 0; pose_matrix(3,1) = 0;
	pose_matrix(0,2) = 0; pose_matrix(1,2) = 0; pose_matrix(2,2) = 1; pose_matrix(3,2) = 0;
	pose_matrix(0,3) = 0; pose_matrix(1,3) = 0; pose_matrix(2,3) = 0; pose_matrix(3,3) = 1;
	
	
	mapping::SLAMGraph graph;
	int graphAddition = 0;
	
	cv::Mat first_image;
	while(ros::ok())
	{
		//IMAGE MATCHING
		
		//~ if(frameNum >= 100 && firstFrame)
		//~ {
			//~ first_image = *(current_color_frame_.second);
			//~ firstFrame = false;
		//~ }
		
		//~ if(!firstFrame)
		//~ {
			//~ match_and_display(first_image, *current_color_frame_.second);
		//~ }

		//-IMAGE MATCHING
		
		//POINTCLOUD PROCESSING
		if(product_valid)
		{
			if(pointclouds.size() > 0)
			{
				//~ ROS_INFO("Processing pointcloud");
				production_possible = false;
				std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud = pointclouds.front();
				pointclouds.pop_front();
				
				while(pointclouds.size() > 0)
					pointclouds.pop_front();
				
				if(first_pointcloud)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_target(new pcl::PointCloud<pcl::PointXYZ>);
					
					sparse_ification(current_pointcloud.second, sparse_input, 500);
					//~ sparse_ification(first_pointcloud, sparse_target);
					
					transform_cloud_by_matrix(sparse_input, pose_matrix);
					
					icp_matcher.setInputSource(sparse_input);
					
					std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>> candidate_clouds;
					int backlog = 10;
					Eigen::Matrix3f rotation1;
					rotation1(0,0) = pose_matrix(0,0); rotation1(1,0) = pose_matrix(1,0); rotation1(2,0) = pose_matrix(2,0);
					rotation1(0,1) = pose_matrix(0,1); rotation1(1,1) = pose_matrix(1,1); rotation1(2,1) = pose_matrix(2,1);
					rotation1(0,2) = pose_matrix(0,2); rotation1(1,2) = pose_matrix(1,2); rotation1(2,2) = pose_matrix(2,2);
					
					for(std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>>::iterator it = transformed_pointclouds.begin(); it != transformed_pointclouds.end(); it++)
					{
						if(candidate_clouds.size() == backlog)
						{
							break;
						}
						
						Eigen::Matrix3f rotation2;
						rotation2(0,0) = (*it).first(0,0); rotation2(1,0) = (*it).first(1,0); rotation2(2,0) = (*it).first(2,0);
						rotation2(0,1) = (*it).first(0,1); rotation2(1,1) = (*it).first(1,1); rotation2(2,1) = (*it).first(2,1);
						rotation2(0,2) = (*it).first(0,2); rotation2(1,2) = (*it).first(1,2); rotation2(2,2) = (*it).first(2,2);
						
						Eigen::Quaternionf q1;
						q1 = rotation1;
						Eigen::Quaternionf q2;
						q2 = rotation2;
						
						double dist = pow(q1.x() - q2.x(),2) + pow(q1.y()- q2.y(),2) + pow(q1.z() - q2.z(),2) + pow(q1.w() - q2.w(),2);
						dist += pow(pose_matrix(3,0) - (*it).first(3,0),2) + pow(pose_matrix(3,1) - (*it).first(3,1),2) + pow(pose_matrix(3,2) - (*it).first(3,2),2);
						
						if(dist < 3.0e-3)
						{
							if(candidate_clouds.size() < backlog)
							{
								candidate_clouds.push_back(*it);
							}
						}
						
					}
					
					candidate_clouds.push_front(*(transformed_pointclouds.end()-1));
					
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZ>);
					Eigen::Matrix4f transform_matrix;
					double fitness = 100;
					
					for(std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>>::reverse_iterator it = candidate_clouds.rbegin(); it != candidate_clouds.rend() && it != candidate_clouds.rbegin()+backlog; it++)
					{
						
						icp_matcher.setInputTarget((*it).second);
						
						icp_matcher.align(*cloud_source_registered);
						
						if(icp_matcher.getFitnessScore() < fitness)
						{
							transform_matrix = icp_matcher.getFinalTransformation();
							fitness = icp_matcher.getFitnessScore();
							sparse_target = (*it).second;
						}
						
						//~ std::cout << fitness << std::endl;
						
						if(fitness < 2.0e-2)
						{
							break;
						}
						
					}
					
					publishCloud2(*sparse_target);
					publishCloud3(*cloud_source_registered);
					
					if(icp_matcher.getFitnessScore() <= 2.0e-2)
					{
						pose_matrix = transform_matrix * pose_matrix;
						transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(pose_matrix, cloud_source_registered));
						map_pointcloud += *cloud_source_registered;
						
						//~ publishCloud1(map_pointcloud);
						if(graphAddition % 10 == 0)
						{
							std::unique_lock<std::mutex> lock_for_scope1(color_image_mutex);
							std::vector<cv::KeyPoint> keypoints;
							cv::Mat descriptors;
							color_feature_generator->detectAndCompute(*(current_color_frame_.second), cv::noArray(), keypoints, descriptors);
							
							if(descriptors.type() != CV_32F)
							{
								descriptors.convertTo(descriptors, CV_32F);
							}
							
							std::shared_ptr<mapping::SLAMNode> newNode(new mapping::SLAMNode(transformed_pointclouds.back().second
								, transformed_pointclouds.back().first
								, *(current_color_frame_.second)
								, keypoints
								, descriptors));
							
							int correspondences = graph.find_corresponding(*newNode, 5).size();
							
							graph.addNode(newNode);
							
							std::cout << "adding new node with " << correspondences << " correspondences" << std::endl;
						}
					}
				}
				else
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr sparse(new pcl::PointCloud<pcl::PointXYZ>);
					sparse_ification(current_pointcloud.second, sparse, 1000);
					first_pointcloud = sparse;
					Eigen::Matrix4f transform_matrix;
					transform_matrix(0,0) = 1; transform_matrix(1,0) = 0; transform_matrix(2,0) = 0; transform_matrix(3,0) = 0;
					transform_matrix(0,1) = 0; transform_matrix(1,1) = 1; transform_matrix(2,1) = 0; transform_matrix(3,1) = 0;
					transform_matrix(0,2) = 0; transform_matrix(1,2) = 0; transform_matrix(2,2) = 1; transform_matrix(3,2) = 0;
					transform_matrix(0,3) = 0; transform_matrix(1,3) = 0; transform_matrix(2,3) = 0; transform_matrix(3,3) = 1;
					transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(transform_matrix, sparse));
					//~ ROS_INFO("First Pointcloud - nothing to do");
				}
				
				production_possible = true;
				producer_notification.notify_all();
				graphAddition += 1;
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
		//-POINTCLOUD PROCESSING
	}
}
