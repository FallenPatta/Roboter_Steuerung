#ifndef _OCTREEMAPPER_H_
#define _OCTREEMAPPER_H_

#include "SLAM/Mapper.h"

#include <iostream>
#include <vector>
#include <deque>
#include <memory>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_pointcloud.h>

namespace mapper{

class OctreeMapper : public Mapper{
	
	private:
	
		std::shared_ptr<pcl::octree::OctreePointCloud<pcl::PointXYZ>> octree_pointer;
		
	public:
	
	OctreeMapper(){
		
		octree_pointer.reset(new pcl::octree::OctreePointCloud<pcl::PointXYZ>(0.1f));
		
	}
	
	virtual ~OctreeMapper(){

	}
	
	virtual bool add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	
	virtual bool add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::Pose);
};

} //mapper

#endif //_OCTREEMAPPER_H_
