#ifndef _POSEESTIMATOR_H_
#define _POSEESTIMATOR_H_

#include <iostream>
#include <vector>
#include <deque>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace estimator{

class PoseEstimator{
	
	public:
	
	/**
	 * Pose Estimator Constructor
	 */
	PoseEstimator(){
		
	}
	
	virtual ~PoseEstimator();
	
	/**
	 * Input: IMU Data
	 * Output: Best Pose Estimate
	 */
	virtual bool update_from_imu(sensor_msgs::Imu& input, geometry_msgs::PoseWithCovariance& result) = 0;
	
	/**
	 * Input: Pose Estimation with Covariance
	 * Output: Best Pose Estimate
	 */
	virtual bool update_from_localization(geometry_msgs::PoseWithCovariance& input, geometry_msgs::PoseWithCovariance& result) = 0;
	
	/**
	 * Returns the current best estimation of the Pose
	 */
	virtual geometry_msgs::PoseWithCovariance get_current_best_state() = 0;
};

} // namespace estimator

#endif // _POSEESTIMATOR_H_
