
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

#include "SLAM/FeatureGenerator.h"
#include "SLAM/Localizer.h"
#include "SLAM/Mapper.h"
#include "SLAM/ICP.h"
#include "SLAM/GraphMap.h"

struct PointXYZ{
	float x,y,z,u;
};

int point_transform(PointXYZ* vector_array, const int array_size, float* mat_4x4);
int testmain(float* vector_array, float* result_array, const int array_size, float* mat_4x4);

ros::Subscriber pointcloud_subscriber_;
ros::Subscriber gyroscope_subscriber;
ros::Subscriber accelerometer_subscriber;
ros::Subscriber depth_image_subscriber;
ros::Subscriber color_image_subscriber;

ros::Publisher pointcloud_publisher_;
ros::Publisher pointcloud_publisher_2_;
ros::Publisher pointcloud_publisher_3_;

std::deque<std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr>> pointclouds;

std::deque<std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>> transformed_pointclouds;

std::deque<std::pair<ros::Time, sensor_msgs::Image::Ptr>> depth_images;

std::deque<std::pair<ros::Time, sensor_msgs::Image::Ptr>> color_images;

std::pair<std::shared_ptr<std::vector<cv::KeyPoint>>, std::shared_ptr<std::vector<cv::KeyPoint>>> keypoint_pair;
std::pair<std::shared_ptr<cv::Mat>, std::shared_ptr<cv::Mat>> descriptor_pair;

std::shared_ptr<FeatureGenerator> depth_feature_generator;
std::shared_ptr<FeatureGenerator> color_feature_generator;
const double akaze_color_thresh = 1e-3;
const double akaze_depth_thresh = 1e-4;

std::shared_ptr<cv::Mat> current_depth_frame_;
bool depth_frame_available = false;

std::shared_ptr<cv::Mat> current_color_frame_;
bool color_frame_available = false;

std::mutex producer_mutex_;
std::condition_variable producer_notification;
std::mutex consumer_mutex_;
std::condition_variable consumer_notification;
bool product_valid = false;
bool production_possible = true;

std::mutex keypoint_pair_mutex;
std::mutex descriptor_pair_mutex;

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

//~ void gyroscopeCallback(const sensor_msgs::Imu::ConstPtr & gyroData){
	//~ sensor_msgs::Imu manipulatable_data = *gyroData;
	//~ gyroscope_samples.push_front(std::pair<ros::Time, sensor_msgs::Imu>(gyroData->header.stamp, manipulatable_data));
//~ }

//~ void accelerometerCallback(const sensor_msgs::Imu::ConstPtr & accelData){
	//~ sensor_msgs::Imu manipulatable_data = *accelData;
	//~ accelerometer_samples.push_front(std::pair<ros::Time, sensor_msgs::Imu>(accelData->header.stamp, manipulatable_data));
//~ }

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
	
	//~ msg_pointcloud.is_dense = false;
	msg_pointcloud.header.frame_id = "/camera_depth_optical_frame";
		
	//~ pcl::PCLPointCloud2 cloud;
	
	//~ pcl::toPCLPointCloud2(to_send, cloud);
	//~ pcl_conversions::fromPCL(cloud, msg_pointcloud);
	
	pointcloud_publisher_3_.publish(msg_pointcloud);
}

void depthImageCallback(const sensor_msgs::Image::ConstPtr & depth_image){
	//~ static int num = 0;
	//~ num++;
	//~ ROS_INFO("%d depth images should be in the queue", num);
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(depth_image, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
	
	cv::Mat output = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC1);
	
	// Update GUI Window
	cv::MatIterator_<uint16_t> it, end;
	cv::MatIterator_<uint8_t> visual_it;
	int max = 10000;
	double value = (double)(0xFFFF)/(double)max;
	double visual_multiplier = (double)0xFF/(double)0xFFFF;
	for(it = cv_ptr->image.begin<uint16_t>(), end = cv_ptr->image.end<uint16_t>(), visual_it = output.begin<uint8_t>(); it != end; ++it, ++visual_it)
	{
		if(*it > 10000){
			*it = 10000;
		}
		*it = std::min(0xFFFF, static_cast<int>(value * (*it)));
		*visual_it = std::min(255,(int)(visual_multiplier * (*it)));
	}
	
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	depth_feature_generator->get_keypoints(cv_ptr->image, keypoints, descriptors);
	cv::drawKeypoints(output, keypoints, output, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
	
	{
		std::unique_lock<std::mutex> lock_for_scope(keypoint_pair_mutex);
		if(keypoint_pair.first)
		{
			*keypoint_pair.first = keypoints;
		}
		else
		{
			keypoint_pair.first = std::make_shared<std::vector<cv::KeyPoint>>(keypoints);
		}
	}
	
	{
		std::unique_lock<std::mutex> lock_for_scope(descriptor_pair_mutex);
		if(descriptor_pair.first)
		{
			*descriptor_pair.first = descriptors;
		}
		else
		{
			descriptor_pair.first = std::make_shared<cv::Mat>(descriptors);
		}
	}
	
	depth_frame_available = false;
	current_depth_frame_ = std::make_shared<cv::Mat>(output);
    depth_frame_available = true;
	consumer_notification.notify_one();
}

void colorImageCallback(const sensor_msgs::Image::ConstPtr & color_image){
	//~ static int num = 0;
	//~ num++;
	//~ ROS_INFO("%d color images should be in the queue", num);
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
	
	//~ cv::Mat gray_image;
	//~ cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_RGB2GRAY);
	
	std::vector<cv::KeyPoint> keypoints;
	cv::Mat descriptors;
	color_feature_generator->get_keypoints(cv_ptr->image, keypoints, descriptors);
	cv::drawKeypoints(cv_ptr->image, keypoints, cv_ptr->image, cv::Scalar(255,0,0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    
	{
		std::unique_lock<std::mutex> lock_for_scope(keypoint_pair_mutex);
		if(keypoint_pair.second)
		{
			*keypoint_pair.second = keypoints;
		}
		else
		{
			keypoint_pair.second = std::make_shared<std::vector<cv::KeyPoint>>(keypoints);
		}
	}
	
	{
		std::unique_lock<std::mutex> lock_for_scope(descriptor_pair_mutex);
		if(descriptor_pair.second)
		{
			*descriptor_pair.second = descriptors;
		}
		else
		{
			descriptor_pair.second = std::make_shared<cv::Mat>(descriptors);
		}
	}

	color_frame_available = false;
	current_color_frame_ = std::make_shared<cv::Mat>(cv_ptr->image);
    color_frame_available = true;
	consumer_notification.notify_one();
}

void shutdownThread(){
	ros::Rate rate(1);
	while(ros::ok()){
		rate.sleep();
	}
	product_valid = true;
	consumer_notification.notify_one();
}

void displayRefreshThread(){
	ros::Rate rate(30);
	while(ros::ok()){
		product_valid = true;
		consumer_notification.notify_one();
		rate.sleep();
	}
}

void sparse_ification(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){
	for(pcl::PointXYZ point : *input)
	{
		if(rand() % 500 == 0)
		{
			if(!(point.x == 0 && point.y == 0 && point.z == 0))
				output->push_back(point);
		}
	}
}

void sparse_ification(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output){
	for(pcl::PointXYZ point : *input)
	{
		if(rand() % 100 == 0)
		{
			output->push_back(point);
		}
	}
}

void transform_cloud_by_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix){
	float matrix[16] = {transform_matrix(0,0), transform_matrix(0,1), transform_matrix(0,2), transform_matrix(0,3), 
						transform_matrix(1,0), transform_matrix(1,1), transform_matrix(1,2), transform_matrix(1,3), 
						transform_matrix(2,0), transform_matrix(2,1), transform_matrix(2,2), transform_matrix(2,3), 
						transform_matrix(3,0), transform_matrix(3,1), transform_matrix(3,2), transform_matrix(3,3)};
	
	//~ for(int i = 0; i<16; i++){
		//~ if(i%4==0)
		//~ {
			//~ std::cout << "\n";
		//~ }
		//~ std::cout << matrix[i] << ",";
	//~ }
	//~ std::cout << "\n";
	
	std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> * points = &cloud->points;
	PointXYZ * floated_points = (PointXYZ*)&((*points)[0]);
	point_transform(floated_points, points->size(), matrix);
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
	//~ gyroscope_subscriber = nh.subscribe<sensor_msgs::Imu>("gyro/sample", 100, gyroscopeCallback);
	//~ accelerometer_subscriber = nh.subscribe<sensor_msgs::Imu>("accel/sample", 100, accelerometerCallback);
	//~ depth_image_subscriber = nh.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, depthImageCallback); // /camera/aligned_depth_to_color/image_raw , /camera/depth/image_rect_raw
	//~ color_image_subscriber = nh.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 1, colorImageCallback);
	
	pointcloud_publisher_ = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1);
	pointcloud_publisher_2_ = nh.advertise<sensor_msgs::PointCloud2>("/reference_debug_cloud", 1);
	pointcloud_publisher_3_ = nh.advertise<sensor_msgs::PointCloud2>("/registered_debug_cloud", 1);
	
	//~ cv::Ptr<cv::AKAZE> depth_akaze = cv::AKAZE::create();
	//~ depth_akaze->setThreshold(akaze_depth_thresh);
	//~ cv::Ptr<cv::AKAZE> color_akaze = cv::AKAZE::create();
	//~ color_akaze->setThreshold(akaze_color_thresh);
	//~ depth_feature_generator.reset(new FeatureGenerator(depth_akaze));
	//~ color_feature_generator.reset(new FeatureGenerator(color_akaze));
	
	//~ pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_matcher;
	pcl::IterativeClosestPointNonLinear<pcl::PointXYZ, pcl::PointXYZ> icp_matcher;
	icp_matcher.setTransformationEpsilon (0);
	icp_matcher.setMaxCorrespondenceDistance (0.50);
	icp_matcher.setMaximumIterations (20);
	icp_matcher.setEuclideanFitnessEpsilon (0);
	//~ icp_matcher.setMaxCorrespondenceDistance (1.0);
	//~ icp_matcher.setMaximumIterations (50);
	//~ icp_matcher.setTransformationEpsilon (1e-8);
	//~ icp_matcher.setEuclideanFitnessEpsilon (1e-1);
	
	// Set the max correspondence distance to 10cm (e.g., correspondences with higher distances will be ignored)
	//~ icp_matcher.setMaxCorrespondenceDistance (0.1);
	// Set the maximum number of iterations (criterion 1)
	//~ icp_matcher.setMaximumIterations (50);
	// Set the transformation epsilon (criterion 2)
	//~ icp_matcher.setTransformationEpsilon (1e-8);
	// Set the euclidean distance difference epsilon (criterion 3)
	//~ icp_matcher.setEuclideanFitnessEpsilon (0.1);
	
	ros::AsyncSpinner spinner(4);
	spinner.start();
	
	std::thread shutdown_security(shutdownThread);
	
	std::shared_ptr<localizer::Localizer> localizer_ptr;
	std::shared_ptr<mapper::Mapper> mapper_ptr;
	
	// TODO: Localizer und Mapper in dynamic Reconfigure Funktion austauschbar machen und per Parameter initialisieren
	
	localizer_ptr.reset(new localizer::ICP());
	mapper_ptr.reset(new mapper::GraphMap());
	
	//~ cv::namedWindow("color_image_window");
	//~ cv::namedWindow("depth_image_window");
	//~ cv::namedWindow("Good Matches");
	
	//~ std::thread(displayRefreshThread);
	
	
	bool firstFrame = true;
	pcl::PointCloud<pcl::PointXYZ>::Ptr first_pointcloud;
	while(ros::ok())
	{
		
		//~ {
			//~ std::unique_lock<std::mutex> lock_for_scope1(keypoint_pair_mutex);
			//~ std::unique_lock<std::mutex> lock_for_scope2(descriptor_pair_mutex);
			//~ if(descriptor_pair.first)
				//~ ROS_INFO("%d , %d mat in descriptors.first", descriptor_pair.first->cols, descriptor_pair.first->rows);
			//~ if(descriptor_pair.second)
				//~ ROS_INFO("%d , %d mat in descriptors.second", descriptor_pair.second->cols, descriptor_pair.second->rows);
					
			//~ if(descriptor_pair.first && descriptor_pair.second && !descriptor_pair.first->empty() && !descriptor_pair.second->empty())
			//~ {
				//~ if(descriptor_pair.first->type() != CV_32F)
				//~ {
					//~ (*descriptor_pair.first).convertTo((*descriptor_pair.first), CV_32F);
				//~ }
				//~ if(descriptor_pair.second->type() != CV_32F)
				//~ {
					//~ (*descriptor_pair.second).convertTo((*descriptor_pair.second), CV_32F);
				//~ }
				//~ cv::FlannBasedMatcher matcher;
				//~ std::vector<cv::DMatch> matches;
				//~ matcher.match(*descriptor_pair.first, *descriptor_pair.second, matches);

				//~ double max_dist = 0; double min_dist = 1000;

				//~ for( int i = 0; i < descriptor_pair.first->rows; i++ )
				//~ { 
					//~ double dist = matches[i].distance;
					//~ if( dist < min_dist ) min_dist = dist;
					//~ if( dist > max_dist ) max_dist = dist;
				//~ }
				//~ ROS_INFO("Max dist : %f \n", max_dist );
				//~ ROS_INFO("Min dist : %f \n", min_dist );

				//~ std::vector<cv::DMatch> good_matches;
				//~ for( int i = 0; i < descriptor_pair.first->rows; i++ )
				//~ {
					//~ if( matches[i].distance <= std::min(1.2*min_dist, 10.0) )
					//~ {
						//~ good_matches.push_back( matches[i]);
					//~ }
				//~ }
				//~ ROS_INFO("good matches: %lu", matches.size());
				
				//~ cv::Mat img_matches;
				//~ drawMatches( (*current_depth_frame_), (*keypoint_pair.first), (*current_color_frame_), (*keypoint_pair.second), good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
				//~ //-- Show detected matches
				//~ cv::imshow( "Good Matches", img_matches );
				
			//~ }
		//~ }
		
		//~ ROS_INFO("Number of unprocessed pointclouds: %lu", pointclouds.size());
		if(product_valid)
		{
			if(pointclouds.size() > 0)
			{
				//~ ROS_INFO("Processing pointcloud");
				production_possible = false;
				std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud = pointclouds.front();
				pointclouds.pop_front();
				
				if(first_pointcloud)
				{
					pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_target(new pcl::PointCloud<pcl::PointXYZ>);
					sparse_ification(current_pointcloud.second, sparse_input);
					sparse_ification(first_pointcloud, sparse_target);
					icp_matcher.setInputSource(sparse_input);
					icp_matcher.setInputTarget(first_pointcloud);
					// Perform the alignment
					pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZ>);
					icp_matcher.align(*cloud_source_registered);
					// Obtain the transformation that aligned cloud_source to cloud_source_registered
					Eigen::Matrix4f transform_matrix = icp_matcher.getFinalTransformation();
					
					//~ for(int i = 0; i<10; i++)
					//~ {
						//~ icp_matcher.setInputSource(cloud_source_registered);
						//~ icp_matcher.align(*cloud_source_registered);
						//~ Eigen::Matrix4f tmp = icp_matcher.getFinalTransformation();
						//~ transform_matrix = transform_matrix * tmp;
					//~ }
					
					std::cout << "has converged:" << icp_matcher.hasConverged() << " score: " << icp_matcher.getFitnessScore() << std::endl;
					
					//~ Eigen::Matrix4f inverted = transform_matrix.inverse();
					
					//~ std::cout << transform_matrix << std::endl;
					
					transform_cloud_by_matrix(current_pointcloud.second, transform_matrix);
					//~ transform_cloud_by_matrix(sparse_input, transform_matrix);
					//~ transform_cloud_by_matrix(current_pointcloud.second, inverted);
											
					transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(transform_matrix, current_pointcloud.second));
					
					publishCloud1(*current_pointcloud.second);
					publishCloud2(*first_pointcloud);
					publishCloud3(*cloud_source_registered);
				}
				else
				{
					first_pointcloud = current_pointcloud.second;
					Eigen::Matrix4f transform_matrix;
					transform_matrix(0,0) = 1; transform_matrix(1,0) = 0; transform_matrix(2,0) = 0; transform_matrix(3,0) = 0;
					transform_matrix(0,1) = 0; transform_matrix(1,1) = 1; transform_matrix(2,1) = 0; transform_matrix(3,1) = 0;
					transform_matrix(0,2) = 0; transform_matrix(1,2) = 0; transform_matrix(2,2) = 1; transform_matrix(3,2) = 0;
					transform_matrix(0,3) = 0; transform_matrix(1,3) = 0; transform_matrix(2,3) = 0; transform_matrix(3,3) = 1;
					transformed_pointclouds.push_back(std::pair<Eigen::Matrix4f, pcl::PointCloud<pcl::PointXYZ>::Ptr>(transform_matrix, current_pointcloud.second));
					//~ ROS_INFO("First Pointcloud - nothing to do");
				}
				
				//last_pointcloud = sparse_input;
				production_possible = true;
				producer_notification.notify_all();
				//~ if(firstFrame){
					//~ firstFrame = false;
					
					//~ for(pcl::PointCloud<pcl::PointXYZ>::iterator iter = current_pointcloud.second->begin(); iter != current_pointcloud.second->end(); ++iter)
					//~ {
						//~ std::cout << "(" << (*iter).x << "," << (*iter).y << "," << (*iter).z << ")" << std::endl;
					//~ }
					
					//~ ROS_INFO("Working off Pointcloud of size: %u x %u",current_pointcloud.second->width, current_pointcloud.second->height);
					//~ mapper_ptr->add_cloud(current_pointcloud.second);
				//~ }
			}
			//~ if(color_frame_available && (*current_color_frame_).rows > 0 && (*current_color_frame_).cols > 0)
			//~ {
				//~ cv::imshow("color_image_window", (*current_color_frame_));
			//~ }
			//~ if(depth_frame_available && (*current_depth_frame_).rows > 0 && (*current_depth_frame_).cols > 0)
			//~ {
				//~ cv::imshow("depth_image_window", (*current_depth_frame_));
			//~ }
			//~ cv::waitKey(15);
			if(pointclouds.size() == 0)
			{
				//~ ROS_INFO("All clouds processed");
				product_valid = false;
			}
		}
		else
		{
			//~ ROS_INFO("Waiting");
			std::unique_lock<std::mutex> lock(consumer_mutex_);
			consumer_notification.wait(lock, []{return product_valid;});
		}
	}
}
