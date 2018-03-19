#ifndef _LOCALIZER_H_
#define _LOCALIZER_H_

#include <iostream>
#include <vector>
#include <deque>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace localizer{

class Localizer{
	
	public:
	
	Localizer(){
		
	}
	
	virtual ~Localizer(){
		
	}
	
	virtual bool localize(pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::PoseWithCovariance estimate, geometry_msgs::PoseWithCovariance& result, bool continuous = true) = 0;
};

} // namespace localizer

#endif // _LOCALIZER_H_
