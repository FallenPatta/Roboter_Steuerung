#ifndef SLAMGRAPH_H_
#define SLAMGRAPH_H_

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

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <SLAM/SLAMNode.h>
#include <SLAM/SLAMMatch.h>
#include "/home/david/Desktop/ros_workspace/src/dynamic_library_projects/src3/Util/PointCloudDisplay.hpp"

namespace mapping{
	
	typedef struct{
		std::pair<int,std::shared_ptr<SLAMNode>> c;
		int size = 0;
	}Correspondence;
	
	class SLAMGraph{
		public:
			SLAMGraph();
			virtual ~SLAMGraph();
			void addNode(SLAMNode);
			void addNode(std::shared_ptr<SLAMNode>);
			void removeNode(std::shared_ptr<SLAMNode>);
			Eigen::Matrix4f lastPose();
			void clear();
			int size();
			std::shared_ptr<SLAMNode> getNode(int);
			std::vector<Correspondence> find_corresponding(SLAMNode&, int, int, int after = 0, bool backwards = false);
			//~ bool correct(int closed_from, int closed_to, Eigen::Matrix4f&);
			
			bool addFirstNode(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud, std::pair<ros::Time, std::shared_ptr<cv::Mat>> current_color_frame_);
			bool addNodeFromCloud(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud, std::pair<ros::Time, std::shared_ptr<cv::Mat>> current_color_frame_);
			
			void setDisplayer(PointCloudDisplay *);
			
		protected:
			double calculateLoopError(int, int);
			Eigen::Matrix4f slerp(Eigen::Matrix4f src, Eigen::Matrix4f tar);
			void sparse_ification(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, int constant);
			double getTransformDistance(Eigen::Matrix4f a, Eigen::Matrix4f b);
			
			void generalized_least_square_estimation(std::deque<SLAMMatch> matches, Eigen::Matrix4f& beta, Eigen::Matrix4f& beta_covariance);
			void generalized_least_square_piecewise_estimation(std::deque<SLAMMatch> matches, Eigen::Matrix4f& beta, Eigen::Matrix4f& beta_covariance);
			
			/**
			 * CUDA Spezifische Funktionen
			 * TODO: Unabhängige Varianten schreiben
			 */
			void transform_cloud_by_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix);
		
			/**
			 * node_matcher -> NN Search for Descriptors
			 * color_feature_generator -> for generating keypoints
			 * scan_matcher -> for matching scans
			 */
			cv::FlannBasedMatcher node_matcher;
			cv::Ptr<cv::Feature2D> color_feature_generator;
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> scan_matcher;
			
			/** The mapping epsilon for the icp algorithm
			 * 	TODO: Parametrisieren
			 */
			double mapping_epsilon = 2.0e-2;
			
			/**
			 * The actual Graph scructure
			 */
			std::vector<std::shared_ptr<SLAMNode>> nodes;
			
			/** 
			 * This Matrix contains the currently best pose Estimate
			 */
			Eigen::Matrix4f pose_matrix;
			
			/**
			 * How many Nodes should the local matching step look for?
			 * TODO: Parametrisieren
			 */
			 int cloud_candidate_backlog = 10;
			
			/**
			 * Thresholds for the distance between FLANN Feature Matches to be accepted
			 * TODO: Parametrisieren
			 */
			double min_dist_relative_threshold = 3.0;
			double min_dist_absolute_threshold = 4.0;
			
			/**
			 * Optional display Pointer
			 */
			 PointCloudDisplay * displayer = 0;
	};
	
} //namespace mapping

#endif //SLAMGRAPH_H_
