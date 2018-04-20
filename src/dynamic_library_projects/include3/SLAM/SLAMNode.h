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

enum COLOR{
	red,
	black
};

namespace mapping{
	
	class SLAMNode{
		public:
			SLAMNode();
			virtual ~SLAMNode();
			pcl::PointCloud<pcl::PointXYZ>::Ptr getCloud();
			std::shared_ptr<Eigen::Matrix4f> getPose();
			std::shared_ptr<cv::Mat> getImage();
			std::shared_ptr<std::vector<cv::KeyPoint>> getKeypoints();
			std::shared_ptr<cv::Mat> getDescriptors();
			void setCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr);
			void setPose(std::shared_ptr<Eigen::Matrix4f>);
			void setImage(std::shared_ptr<cv::Mat>);
			void setKeypoints(std::shared_ptr<std::vector<cv::KeyPoint>>);
			void setDescriptors(std::shared_ptr<cv::Mat>);
		protected:
			pcl::PointCloud<pcl::PointXYZ>::Ptr node_cloud_;
			std::shared_ptr<Eigen::Matrix4f> node_pose;
			std::shared_ptr<cv::Mat> node_image;
			std::shared_ptr<std::vector<cv::KeyPoint>> node_keypoints;
			std::shared_ptr<cv::Mat> node_descriptors;
			
			std::vector<std::shared_ptr<SLAMNode*>> connectedNodes;
			COLOR color;
		
	};
	
} //namespace mapping

#endif //SLAMNODE_H_
