#ifndef _KALMANFILTER_H_
#define _KALMANFILTER_H_

#include "SLAM/PoseEstimator.h"
#include "SLAM/KalmanFilter/Pose3x6D.h"

#include <iostream>
#include <vector>
#include <deque>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace estimator{

class KalmanFilter : public PoseEstimator{
	
	public:
	
	KalmanFilter();
	
	virtual ~KalmanFilter();
	
	virtual bool update_from_imu(sensor_msgs::Imu& input, geometry_msgs::PoseWithCovariance& result);
	
	virtual bool update_from_localization(geometry_msgs::PoseWithCovariance& input, geometry_msgs::PoseWithCovariance& result);
	
	virtual geometry_msgs::PoseWithCovariance get_current_best_state();
	
	private:
	
		void predictCovariance(cv::Mat oldCov, cv::Mat& newCov, double deltaT);
		//~ void predictCovarianceWithEnv(cv::Mat oldCov, cv::Mat& newCov, cv::Mat env, double deltaT);
		//~ void predictState(StateVector2D oldState, StateVector2D& predictState, double deltaT);
		//~ void predictStateWithControl(StateVector2D oldState, StateVector2D& predictState, StateVector2D controlState, double deltaT);
		//~ void correctState(StateVector2D aprioriState, StateVector2D predictState, StateVector2D& correctedState
							//~ , StateVector2D measurements, double deltaT
							//~ , cv::Mat aprioriCov, cv::Mat sensorCov, cv::Mat predictedCov, cv::Mat& correctedCov);
};

} // estimator

#endif //_KALMANFILTER_H_
