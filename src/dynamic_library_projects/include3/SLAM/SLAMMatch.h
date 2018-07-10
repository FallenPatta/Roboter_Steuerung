#ifndef SLAMMATCH_H_
#define SLAMMATCH_H_

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

#include <SLAM/SLAMNode.h>


namespace mapping{
	
	/**
	 * This class contains all informations about a SLAM Match
	 * It is not meant as accurate information about aligned scans
	 * but rather as a single sample for statistical analysis of the real scan pose.
	 * As such it does not contain any information about the target scan itself.
	 * Only a transform from the sourch scan and the transform covariance are stored.
	 */
	class SLAMMatch{
		public:
			SLAMMatch();
			
			/**
			 * Constructor with predicted tf and covariances
			 */
			SLAMMatch(std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>);
			
			/**
			 * Constructor with source Pose, predicted tf and covariances
			 */
			SLAMMatch(std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>);
			
			/**
			 * Constructor with predicted tf, a source Node and covariances
			 */
			SLAMMatch(std::shared_ptr<SLAMNode>, std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>, std::shared_ptr<Eigen::Matrix4f>);
			
			/**
			 * Setter/Getter for the source pose
			 */
			void setSourcePose(std::shared_ptr<Eigen::Matrix4f>);
			std::shared_ptr<Eigen::Matrix4f> getSourcePose();
			
			/**
			 * Setter/Getter for the source - calls the setter for the source pose
			 */
			void setSource(std::shared_ptr<SLAMNode>);
			std::shared_ptr<SLAMNode> getSource();
			
			/**
			 * Setter/Getter for the tf
			 * 
			 * sets the inverse tf
			 */
			void setTransform(std::shared_ptr<Eigen::Matrix4f>);
			std::shared_ptr<Eigen::Matrix4f> getTransform();
			
			/**
			 * Getter for the inverse tf
			 */
			std::shared_ptr<Eigen::Matrix4f> getInverseTransform();
			
			/**
			 * Setter for the covariances
			 */
			void setMatchCovariance(std::shared_ptr<Eigen::Matrix4f> trans, std::shared_ptr<Eigen::Matrix4f> rot);
			
			/**
			 * Setter/Getter for the translation covariance
			 */
			Eigen::Matrix4f getTranslationCovariance();
			void setTranslationCovariance(std::shared_ptr<Eigen::Matrix4f> );
			
			/**
			 * Setter/Getter for the rotation covariance
			 */
			Eigen::Matrix4f getRotationCovariance();
			void setRotationCovariance(std::shared_ptr<Eigen::Matrix4f> );
			
		private:
			/** source of the match */
			std::shared_ptr<SLAMNode> source;
			/** pose of the measurement origin */
			std::shared_ptr<Eigen::Matrix4f> source_pose;
			/** transform from source to target (actual measurement) */
			std::shared_ptr<Eigen::Matrix4f> transform;
			/** transform from target to source */
			std::shared_ptr<Eigen::Matrix4f> inverse_transform;
			/** covariance matrix of the transformation */
			std::shared_ptr<Eigen::Matrix4f> translation_covariance;
			/** covariance matrix of the rotation */
			std::shared_ptr<Eigen::Matrix4f> rotation_covariance;
	};
	
} //namespace mapping

#endif //SLAMMATCH_H_
