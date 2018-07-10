#include <SLAM/SLAMMatch.h>

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
	
	SLAMMatch::SLAMMatch(){
		
	}
	
	SLAMMatch::SLAMMatch(std::shared_ptr<SLAMNode> node, std::shared_ptr<Eigen::Matrix4f> tf, std::shared_ptr<Eigen::Matrix4f> translation_cov, std::shared_ptr<Eigen::Matrix4f> rot_cov){
		setSource(node);
		setTransform(tf);
		setTranslationCovariance(translation_cov);
		setRotationCovariance(rot_cov);
	}
	
	/**
	 * Constructor with predicted tf and covariance
	 */
	SLAMMatch::SLAMMatch(std::shared_ptr<Eigen::Matrix4f> tf, std::shared_ptr<Eigen::Matrix4f> translation_cov, std::shared_ptr<Eigen::Matrix4f> rot_cov){
		setTransform(tf);
		setTranslationCovariance(translation_cov);
		setRotationCovariance(rot_cov);
	}
	
	SLAMMatch::SLAMMatch(std::shared_ptr<Eigen::Matrix4f> source_pose, std::shared_ptr<Eigen::Matrix4f> tf, std::shared_ptr<Eigen::Matrix4f> trans_cov , std::shared_ptr<Eigen::Matrix4f> rot_cov){
		setSourcePose(source_pose);
		setTransform(tf);
		setTranslationCovariance(trans_cov);
		setRotationCovariance(rot_cov);
	}
				
	void SLAMMatch::setSource(std::shared_ptr<SLAMNode> source){
		setSourcePose(source->getPose());
		this->source = source;
	}
	
	std::shared_ptr<SLAMNode> SLAMMatch::getSource(){
		return this->source;
	}
	
	void SLAMMatch::setTransform(std::shared_ptr<Eigen::Matrix4f> transform){
		this->inverse_transform = std::make_shared<Eigen::Matrix4f>(transform->inverse());
		this->transform = transform;
	}
	
	std::shared_ptr<Eigen::Matrix4f> SLAMMatch::getTransform(){
		return this->transform;
	}
	
	void SLAMMatch::setSourcePose(std::shared_ptr<Eigen::Matrix4f> source_pose){
		this->source_pose = source_pose;
	}
	
	std::shared_ptr<Eigen::Matrix4f> SLAMMatch::getSourcePose(){
		return this->source_pose;
	}
	
	std::shared_ptr<Eigen::Matrix4f> SLAMMatch::getInverseTransform(){
		return inverse_transform;
	}
	
	void SLAMMatch::setMatchCovariance(std::shared_ptr<Eigen::Matrix4f> trans, std::shared_ptr<Eigen::Matrix4f> rot){
		this->translation_covariance = trans;
		this->rotation_covariance = rot;
	}
	
	void SLAMMatch::setTranslationCovariance(std::shared_ptr<Eigen::Matrix4f> trans){
		this->translation_covariance = trans;
	}
	
	void SLAMMatch::setRotationCovariance(std::shared_ptr<Eigen::Matrix4f> rot){
		this->rotation_covariance = rot;
	}
	
	Eigen::Matrix4f SLAMMatch::getTranslationCovariance(){
		if(this->source){
			return (*(this->translation_covariance) + (*(this->source->getTranslationCovariance())));
		}
		else{
			return *(this->translation_covariance);
		}
	}
	
	Eigen::Matrix4f SLAMMatch::getRotationCovariance(){
		if(this->source){
			return (*(this->rotation_covariance) + (*(this->source->getRotationCovariance())));
		}
		else{
			return *(this->rotation_covariance);
		}
	}
	
	//~ std::shared_ptr<Eigen::Matrix4f> SLAMMatch::getMatchCovariance(){
		//~ return this->transform_covariance;
	//~ }

} //namespace mapping
