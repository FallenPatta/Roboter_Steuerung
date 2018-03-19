#ifndef _ICPLOCALIZER_H_
#define _ICPLOCALIZER_H_

#include "SLAM/Localizer.h"

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

class ICP : public Localizer{
	
	private:
	
	
	
	public:
	
	ICP(){
		
	}
	
	virtual ~ICP(){
		
	}
	
	virtual bool localize(pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::PoseWithCovariance estimate, geometry_msgs::PoseWithCovariance& result, bool continuous = true);
};

} // localizer

#endif //_ICPLOCALIZER_H_
