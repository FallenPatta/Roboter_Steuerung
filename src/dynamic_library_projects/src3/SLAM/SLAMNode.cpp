#include <SLAM/SLAMNode.h>

#include <iostream>
#include <vector>
#include <utility>
#include <deque>
#include <memory>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

namespace mapping{
	
	SLAMNode::SLAMNode(){
		keypoint_vector[0] = 0;
		keypoint_vector[1] = 0;
	}
	
	SLAMNode::SLAMNode(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
						, Eigen::Matrix4f pose
						, cv::Mat image
						, std::vector<cv::KeyPoint> keypoints
						, cv::Mat descriptors)
						:
						node_cloud(cloud)
						, node_pose(std::make_shared<Eigen::Matrix4f>(pose))
						, node_image(std::make_shared<cv::Mat>(image))
						, node_keypoints(std::make_shared<std::vector<cv::KeyPoint>>(keypoints))
						, node_descriptors(std::make_shared<cv::Mat>(descriptors))
	{
		keypoint_vector[0] = 0;
		keypoint_vector[1] = 0;
		for(cv::KeyPoint k : *node_keypoints)
		{
			keypoint_vector[0] += cos(k.angle);
			keypoint_vector[1] += sin(k.angle);
		}
		keypoint_unit = sqrt(pow(keypoint_vector[0],2) + pow(keypoint_vector[1],2));
		setTranslationCovariance(std::make_shared<Eigen::Matrix4f>(Eigen::Matrix4f::Zero()));
		setRotationCovariance(std::make_shared<Eigen::Matrix4f>(Eigen::Matrix4f::Zero()));
	}
	
	SLAMNode::~SLAMNode(){
		
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr SLAMNode::getCloud(){
		return node_cloud;
	}
	
	std::shared_ptr<Eigen::Matrix4f> SLAMNode::getPose(){
		return node_pose;
	}
	
	std::shared_ptr<Eigen::Matrix4f> SLAMNode::getTranslationCovariance(){
		return node_translation_covariance;
	}
	
	std::shared_ptr<Eigen::Matrix4f> SLAMNode::getRotationCovariance(){
		return node_rotation_covariance;
	}
	
	std::shared_ptr<cv::Mat> SLAMNode::getImage(){
		return node_image;
	}
	
	std::shared_ptr<std::vector<cv::KeyPoint>> SLAMNode::getKeypoints(){
		return node_keypoints;
	}
	
	std::shared_ptr<cv::Mat> SLAMNode::getDescriptors(){
		return node_descriptors;
	}
	
	void SLAMNode::setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
		node_cloud = cloud;
	}
	
	void SLAMNode::setPose(std::shared_ptr<Eigen::Matrix4f> pose){
		node_pose = pose;
	}
	
	void SLAMNode::setTranslationCovariance(std::shared_ptr<Eigen::Matrix4f> pose_covariance){
		node_translation_covariance = pose_covariance;
	}
	
	void SLAMNode::setRotationCovariance(std::shared_ptr<Eigen::Matrix4f> pose_covariance){
		node_rotation_covariance = pose_covariance;
	}
	
	void SLAMNode::setImage(std::shared_ptr<cv::Mat> image){
		node_image = image;
	}
	
	void SLAMNode::setKeypoints(std::shared_ptr<std::vector<cv::KeyPoint>> keypoints){
		node_keypoints = keypoints;
		keypoint_vector[0] = 0;
		keypoint_vector[1] = 0;
		for(cv::KeyPoint k : *node_keypoints)
		{
			keypoint_vector[0] += cos(k.angle);
			keypoint_vector[1] += sin(k.angle);
		}
		keypoint_unit = sqrt(pow(keypoint_vector[0],2) + pow(keypoint_vector[1],2));
	}
	
	void SLAMNode::setDescriptors(std::shared_ptr<cv::Mat> descriptors){
		node_descriptors = descriptors;
	}


} //namespace mapping
