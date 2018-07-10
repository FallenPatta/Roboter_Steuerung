#ifndef SLAMNODE_H_
#define SLAMNODE_H_

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
	
	//~ enum COLOR{
		//~ red,
		//~ black
	//~ };
	
	class SLAMNode{
		public:
			SLAMNode();
			SLAMNode(pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::Matrix4f, cv::Mat, std::vector<cv::KeyPoint>, cv::Mat);
			virtual ~SLAMNode();
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
			std::shared_ptr<Eigen::Matrix4f> getPose();
			std::shared_ptr<Eigen::Matrix4f> getTranslationCovariance();
			std::shared_ptr<Eigen::Matrix4f> getRotationCovariance();
			std::shared_ptr<cv::Mat> getImage();
			std::shared_ptr<std::vector<cv::KeyPoint>> getKeypoints();
			std::shared_ptr<cv::Mat> getDescriptors();
			
			void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
			void setPose(std::shared_ptr<Eigen::Matrix4f>);
			void setTranslationCovariance(std::shared_ptr<Eigen::Matrix4f>);
			void setRotationCovariance(std::shared_ptr<Eigen::Matrix4f>);
			void setImage(std::shared_ptr<cv::Mat>);
			void setKeypoints(std::shared_ptr<std::vector<cv::KeyPoint>>);
			void setDescriptors(std::shared_ptr<cv::Mat>);
			
		protected:
			pcl::PointCloud<pcl::PointXYZ>::Ptr node_cloud;
			std::shared_ptr<Eigen::Matrix4f> node_pose;
			std::shared_ptr<Eigen::Matrix4f> node_translation_covariance;
			std::shared_ptr<Eigen::Matrix4f> node_rotation_covariance;
			std::shared_ptr<cv::Mat> node_image;
			std::shared_ptr<std::vector<cv::KeyPoint>> node_keypoints;
			std::shared_ptr<cv::Mat> node_descriptors;
			
			double keypoint_vector[2];
			double keypoint_unit;
			
			std::deque<std::shared_ptr<SLAMNode*>> neighbours;
			//~ COLOR color;
		
	};
	
} //namespace mapping

#endif //SLAMNODE_H_
