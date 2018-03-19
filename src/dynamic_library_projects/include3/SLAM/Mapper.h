#ifndef _MAPPER_H_
#define _MAPPER_H_

#include <iostream>
#include <vector>
#include <deque>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace mapper{

class Mapper{
	
	public:
	
	Mapper(){
	
	}
	
	virtual ~Mapper(){
		
	}
	
	virtual bool add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr) = 0;
	
	virtual bool add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::Pose) = 0;
};

} // namespace mapper

#endif // _MAPPER_H_
