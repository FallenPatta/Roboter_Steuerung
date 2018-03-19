#ifndef _POSE3X6D_H_
#define _POSE3X6D_H_

#include <geometry_msgs/PoseWithCovariance.h>

class Pose3x{
	
	private:
	
	geometry_msgs::PoseWithCovariance pose_x;
	geometry_msgs::PoseWithCovariance pose_d_x;
	geometry_msgs::PoseWithCovariance pose_d_d_x;
	
	public:
	
	void set_x(geometry_msgs::PoseWithCovariance);
	geometry_msgs::PoseWithCovariance get_x();
	
	void set_d_x(geometry_msgs::PoseWithCovariance);
	geometry_msgs::PoseWithCovariance get_d_x();
	
	void set_d_d_x(geometry_msgs::PoseWithCovariance);
	geometry_msgs::PoseWithCovariance get_d_d_x();
	
};

#endif //_POSE3X6D_H_
