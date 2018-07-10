
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

#include <Eigen/Geometry>
#include <Eigen/Core>

#include "SLAM/FeatureGenerator.h"
#include "SLAM/Localizer.h"
#include "SLAM/Mapper.h"
#include "SLAM/ICP.h"
#include "SLAM/GraphMap.h"
#include <SLAM/SLAMNode.h>
#include <SLAM/SLAMGraph.h>

#include "Util/MatrixInterpolator.h"
#include "/home/david/Desktop/ros_workspace/src/dynamic_library_projects/src3/Util/PointCloudDisplay.hpp"

typedef pcl::PointXYZ PointType;

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

Eigen::Affine3d depth_camera_fixed_frame;

std::pair<ros::Time, std::shared_ptr<cv::Mat>> current_color_frame_;
int frameNum = 0;

//~ std::shared_ptr<FeatureGenerator> color_feature_generator;
//~ cv::Ptr<cv::AKAZE> color_feature_generator;
cv::Ptr<cv::Feature2D> color_feature_generator;
const double akaze_color_thresh = 1e-4;

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
	//~ pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

	// Convert to PCL data type
	pcl_conversions::toPCL(*msg_pointcloud, *cloud); // "moveFromPCL" could be used if msg_pointcloud wasn't "const"
	
	pcl::fromPCLPointCloud2(*cloud, *standard_cloud);
	
	//Filter NAN Values
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(standard_cloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr result(new pcl::PointCloud<pcl::PointXYZ>());
	pass.filter(*result);
	
	pointclouds.push_back(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>(msg_pointcloud->header.stamp, result));
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

void depth_camera_fixed_frame_callback(sensor_msgs::CameraInfo){

}

void shutdownThread(){
	ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
	}
	product_valid = true;
	consumer_notification.notify_one();
}

void copy_keypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, std::vector<int> keypoint_indices){
	int num = 0;
	for(int ind : keypoint_indices)
	{
		pcl::PointXYZ point = input->points[ind];
		if(!(point.x == 0 && point.y == 0 && point.z == 0))
		{
			output->push_back(point);
		}
	}
}

void sparse_ification(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, int constant, bool center){
	int num = 0;
	if(center && input->width > 100 && input->height > 100)
	{
		double factor = 1.0 + (20.0/input->width);
		int constant_mult = constant * factor;
		for(int x = 10; x < input->width-10; x++)
		{
			for(int y = 10; y < input->height-10; y++)
			{
				num++;
				if(num % constant_mult == 0)
				{
					pcl::PointXYZ point = input->at(x,y);
					if(!(point.x == 0 && point.y == 0 && point.z == 0))
						output->push_back(point);
					num += ( rand() % constant_mult - (constant_mult/2) );
				}
			}
		}
	}
	else
	{
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

double getTransformDistance(Eigen::Matrix4f a, Eigen::Matrix4f b){
	
	Eigen::Vector3f charvec1;
	charvec1(0) = a(0,0) + a(0,1) + a(0,2);
	charvec1(1) = a(1,0) + a(1,1) + a(1,2);
	charvec1(2) = a(2,0) + a(2,1) + a(2,2);
	Eigen::Vector3f trans1;
	trans1(0) = a(0,3);
	trans1(1) = a(1,3);
	trans1(2) = a(2,3);
	//~ trans1 = trans1 / sqrt(trans1.norm());
	
	Eigen::Vector3f charvec2;
	charvec2(0) = b(0,0) + b(0,1) + b(0,2);
	charvec2(1) = b(1,0) + b(1,1) + b(1,2);
	charvec2(2) = b(2,0) + b(2,1) + b(2,2);
	Eigen::Vector3f trans2;
	trans2(0) = b(0,3);
	trans2(1) = b(1,3);
	trans2(2) = b(2,3);
	//~ trans2 = trans2 / sqrt(trans2.norm());
	
	double dist = pow(charvec1.x() - charvec2.x(),2) + pow(charvec1.y()- charvec2.y(),2) + pow(charvec1.z() - charvec2.z(),2);
	dist += pow(trans1.x() - trans2.x(),2) + pow(trans1.y() - trans2.y(),2) + pow(trans1.z() - trans2.z(),2);
	
	//~ dist = sqrt(dist);
	
	return dist;
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
	
	if(mindist > 0.05){
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
	tf::TransformListener camera_tf_listener; //listener for camera transform
		
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
	icp_matcher.setMaximumIterations(7);
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
	Eigen::Matrix4f pose_matrix;
	pose_matrix(0,0) = 1; pose_matrix(1,0) = 0; pose_matrix(2,0) = 0; pose_matrix(3,0) = 0;
	pose_matrix(0,1) = 0; pose_matrix(1,1) = 1; pose_matrix(2,1) = 0; pose_matrix(3,1) = 0;
	pose_matrix(0,2) = 0; pose_matrix(1,2) = 0; pose_matrix(2,2) = 1; pose_matrix(3,2) = 0;
	pose_matrix(0,3) = 0; pose_matrix(1,3) = 0; pose_matrix(2,3) = 0; pose_matrix(3,3) = 1;
	
	
	mapping::SLAMGraph graph;
	int graphAddition = 0;
	bool lost = false;
	int lastcorrection = 0;
	
	double mapping_epsilon = 2.0e-2;
	double lost_epsilon = 10.0 * mapping_epsilon;
	int cloud_candidate_backlog = 5; //how many clouds to check before the most recent cloud is defaulted to
	
	PointCloudDisplay displayer(&pointcloud_publisher_, &pointcloud_publisher_2_, &pointcloud_publisher_3_);
	graph.setDisplayer(&displayer);
	
	cv::Mat first_image;
	while(ros::ok())
	{
		
		//POINTCLOUD PROCESSING
		if(product_valid)
		{
			if(pointclouds.size() > 0)
			{
				//Lock the color Image
				std::unique_lock<std::mutex> lock_for_scope1(color_image_mutex);
				
				//Interrupt the production
				production_possible = false;
				
				//Extract last captured cloud
				std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud = pointclouds.front();
				pointclouds.pop_front();
				
				//clear clouds
				while(pointclouds.size() > 0)
					pointclouds.pop_front();
					
				if(!graph.addNodeFromCloud(current_pointcloud, current_color_frame_)){
					std::cout << "ERROR - Cloud was not added to Graph" << std::endl;
				}
				else{
					std::cout << "Cloud added" << std::endl;
				}
				
				/*
				//if first cloud is set
				if(first_pointcloud)
				{
					if(!lost)
					{
						//Generate input and target for icp
						pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input(new pcl::PointCloud<pcl::PointXYZ>);
						pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_target(new pcl::PointCloud<pcl::PointXYZ>);
						
						//sparseify last captured cloud
						sparse_ification(current_pointcloud.second, sparse_input, 1000, true);
					
						//transform using GPU
						transform_cloud_by_matrix(sparse_input, pose_matrix);
						
						//set the source as the last captured sparse cloud
						icp_matcher.setInputSource(sparse_input);
						
						//list of matching cloud candidates
						std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>> candidate_clouds;
						//~ Eigen::Matrix3f rotation1; //current rotation
						//~ rotation1(0,0) = pose_matrix(0,0); rotation1(1,0) = pose_matrix(1,0); rotation1(2,0) = pose_matrix(2,0);
						//~ rotation1(0,1) = pose_matrix(0,1); rotation1(1,1) = pose_matrix(1,1); rotation1(2,1) = pose_matrix(2,1);
						//~ rotation1(0,2) = pose_matrix(0,2); rotation1(1,2) = pose_matrix(1,2); rotation1(2,2) = pose_matrix(2,2);
						
						//Iterate over all captured clouds -> inefficient! change
						for(std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>>::iterator it = transformed_pointclouds.begin(); it != transformed_pointclouds.end(); it++)
						{
							//how many got?
							if(candidate_clouds.size() == cloud_candidate_backlog)
							{
								break;
							}

							double dist = getTransformDistance(pose_matrix, (*it).first);
							
							//Select closest clouds
							if(dist < 3.0e-3)
							{
								if(candidate_clouds.size() < cloud_candidate_backlog)
								{
									candidate_clouds.push_back(*it);
								}
							}
							
						}
						
						//Add the last added Cloud to the candidates
						candidate_clouds.push_front(transformed_pointclouds.back());
						
						//Init the registered cloud and transform between candidate and registered cloud
						pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZ>);
						Eigen::Matrix4f transform_matrix;
						double fitness = 100;
						
						//Match for all candidates
						for(std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>>::reverse_iterator it = candidate_clouds.rbegin(); it != candidate_clouds.rend() && it != candidate_clouds.rbegin()+cloud_candidate_backlog; it++)
						{
							//Set candidate as target
							icp_matcher.setInputTarget((*it).second);
							
							//Align source and target
							icp_matcher.align(*cloud_source_registered);
							
							//Get fitness
							if(icp_matcher.getFitnessScore() < fitness)
							{
								transform_matrix = icp_matcher.getFinalTransformation();
								fitness = icp_matcher.getFitnessScore();
								sparse_target = (*it).second;
							}
													
							//If good enough -> stop
							if(fitness < mapping_epsilon)
							{
								break;
							}
							
						}
						
						//Publish
						publishCloud2(*sparse_target);
						publishCloud3(*cloud_source_registered);
						
						//Good enough? And Converged? Append!
						if(icp_matcher.getFitnessScore() <= mapping_epsilon && icp_matcher.hasConverged())
						{
							//Calculate current pose
							pose_matrix = transform_matrix * pose_matrix;
							
							//Generate map addition from whole cloud
							pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input_storable(new pcl::PointCloud<pcl::PointXYZ>);
							sparse_ification(current_pointcloud.second, sparse_input_storable, 1000, false);
							
							//transform using GPU
							transform_cloud_by_matrix(sparse_input_storable, pose_matrix);
							
							//Append bigger Cloud
							transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(pose_matrix, sparse_input_storable));
							
							//Compose and add Graph Node
							//if(graphAddition % 10 == 0)
							//{
								
							//Generate Keypoints for current Cloud
							std::vector<cv::KeyPoint> keypoints;
							cv::Mat descriptors;
							color_feature_generator->detectAndCompute(*(current_color_frame_.second), cv::noArray(), keypoints, descriptors);
							
							//Convert descriptors
							if(descriptors.type() != CV_32F)
							{
								descriptors.convertTo(descriptors, CV_32F);
							}
							
							//Init SLAMNode for Graph
							std::shared_ptr<mapping::SLAMNode> newNode(new mapping::SLAMNode(
								transformed_pointclouds.back().second
								, transformed_pointclouds.back().first
								, *(current_color_frame_.second)
								, keypoints
								, descriptors));
							
							//Find correspondences with 10 or more matches
							std::vector<mapping::Correspondence> correspondences = graph.find_corresponding(*newNode, 35, 5);
							
							//Add Node to Graph AFTER matching
							graph.addNode(newNode);
							
							if(correspondences.size() > 0)
							{
								double min_dist = 10000;
								mapping::Correspondence best_corresp;
								for(mapping::Correspondence correspondence : correspondences)
								{
									double dist = getTransformDistance(pose_matrix, *(correspondence.c.second->getPose()));
									//std::cout << "distance: " << dist << std::endl;
									if(min_dist > dist)
									{
										min_dist = dist;
										best_corresp = correspondence;
									}
								}
								
								//Display best correspondence
								display(*(best_corresp.c.second->getImage()), *(current_color_frame_.second)
								, *(best_corresp.c.second->getDescriptors()), *(newNode->getDescriptors())
								, *(best_corresp.c.second->getKeypoints()), *(newNode->getKeypoints()));
								
								//############################################################
								//TEST!
								//############################################################
								//~ if(min_dist > 0.1)
								//~ {
									//~ std::cout << "correcting" << std::endl;
									//~ Eigen::Matrix4f final_pose;
									//~ bool success = graph.correct(graph.size()-1, best_corresp.c.first, final_pose);
									//~ if(success)
									//~ {
										//~ pose_matrix = final_pose;
										
										//~ //Append to map
										//~ pcl::PointCloud<pcl::PointXYZ> map_pointcloud;
										//~ for(int i = best_corresp.c.first; i<graph.size()-1; i++)
										//~ {
											//~ map_pointcloud += *(graph.getNode(i)->getCloud());
										//~ }
										//~ //Publish map
										//~ publishCloud1(map_pointcloud);
									//~ }
									//~ else
									//~ {
										//~ std::cout << "not corrected" << std::endl;
									//~ }
									
								//~ }
								//############################################################
								//############################################################
							}
							else
							{
								cv::waitKey(1);
							}
							
							//	std::cout << "adding new node with " << correspondences.size() << " correspondences" << std::endl;
							//}
							//graphAddition += 1;
							//std::cout << "mapping: " << icp_matcher.getFitnessScore() << std::endl;
						}
						else if(icp_matcher.getFitnessScore() > mapping_epsilon && icp_matcher.getFitnessScore() < lost_epsilon)
						{
							//std::cout << "tracking: " << icp_matcher.getFitnessScore() << std::endl;
						}
						else if (icp_matcher.getFitnessScore() >= lost_epsilon)
						{
							//std::cout << "lost: " << icp_matcher.getFitnessScore() << std::endl;
							lost = true;
						}
					}
					else
					{
						//RECOVERY
						
						//Generate Keypoints
						std::vector<cv::KeyPoint> keypoints;
						cv::Mat descriptors;
						color_feature_generator->detectAndCompute(*(current_color_frame_.second), cv::noArray(), keypoints, descriptors);
						
						//Convert descriptors
						if(descriptors.type() != CV_32F)
						{
							descriptors.convertTo(descriptors, CV_32F);
						}
						
						
						//Init Node for Searching
						std::shared_ptr<mapping::SLAMNode> recoveryNode(new mapping::SLAMNode(
							transformed_pointclouds.back().second
							, transformed_pointclouds.back().first
							, *(current_color_frame_.second)
							, keypoints
							, descriptors));
						
						//Search for Correspondences
						std::vector<mapping::Correspondence> correspondences = graph.find_corresponding(*recoveryNode, 20, 1000);
						
						if(correspondences.size() > 0)
						{
							//Find best
							int max_ind = 0;
							int max_cor = 0;
							for(int i = 0; i < correspondences.size(); i++)
							{
								std::pair<int,std::shared_ptr<mapping::SLAMNode>> p = correspondences[i].c;
								if(p.first > max_cor)
								{
									max_cor = p.first;
									max_ind = i;
								}
							}
							
							//Select best
							std::shared_ptr<mapping::SLAMNode> selected_node = correspondences[max_ind].c.second;
							
							//Reset Pose
							Eigen::Matrix4f tmp = pose_matrix;
							pose_matrix = *selected_node->getPose();

							//Set input and target for icp
							pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input(new pcl::PointCloud<pcl::PointXYZ>);
							
							//sparseify last captured cloud
							sparse_ification(current_pointcloud.second, sparse_input, 1000);
							transform_cloud_by_matrix(sparse_input, pose_matrix);
							
							//Set input of best Correspondence
							icp_matcher.setInputSource(sparse_input);
							icp_matcher.setInputTarget(selected_node->getCloud());
							
							//Align
							pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZ>);
							icp_matcher.align(*cloud_source_registered);
							
							//IF align is good -> set new Pose
							if(icp_matcher.getFitnessScore() < mapping_epsilon && icp_matcher.hasConverged())
							{
								pose_matrix = icp_matcher.getFinalTransformation() * pose_matrix;
								//transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(pose_matrix, cloud_source_registered));
								std::cout << "recovering" << std::endl;
								lost = false;
							}
							else
							{
								//Reset Pose
								pose_matrix = tmp;
								std::cout << "could not recover" << std::endl;
							}
						}
					}
					
				}
				else
				{
					//If first frame
					pcl::PointCloud<pcl::PointXYZ>::Ptr sparse(new pcl::PointCloud<pcl::PointXYZ>);
					sparse_ification(current_pointcloud.second, sparse, 1000);
					//Set first frame
					first_pointcloud = sparse;
					//Create unit matrix
					Eigen::Matrix4f transform_matrix;
					transform_matrix(0,0) = 1; transform_matrix(1,0) = 0; transform_matrix(2,0) = 0; transform_matrix(3,0) = 0;
					transform_matrix(0,1) = 0; transform_matrix(1,1) = 1; transform_matrix(2,1) = 0; transform_matrix(3,1) = 0;
					transform_matrix(0,2) = 0; transform_matrix(1,2) = 0; transform_matrix(2,2) = 1; transform_matrix(3,2) = 0;
					transform_matrix(0,3) = 0; transform_matrix(1,3) = 0; transform_matrix(2,3) = 0; transform_matrix(3,3) = 1;
					//Push into list
					transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(transform_matrix, sparse));
					//ROS_INFO("First Pointcloud - nothing to do");
					
					tf::StampedTransform transform;
					try{
						camera_tf_listener.lookupTransform("/camera_depth_frame", "/camera_link", ros::Time(0), transform);
						tf::transformTFToEigen(transform, depth_camera_fixed_frame);
					}
					catch (tf::TransformException ex){
						ROS_ERROR("%s",ex.what());
					}
				}
				*/
				
				production_possible = true;
				producer_notification.notify_all();
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
