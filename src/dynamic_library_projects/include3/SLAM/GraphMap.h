#ifndef _GRAPHMAP_H_
#define _GRAPHMAP_H_

#include "SLAM/Mapper.h"

#include <iostream>
#include <vector>
#include <deque>
#include <memory>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace mapper{

class GraphMap : public Mapper{
	
	private:
	
		std::vector<std::shared_ptr<std::pair<geometry_msgs::Pose, pcl::PointCloud<pcl::PointXYZ>::Ptr>>> pointcloud_fingerprints;
		pcl::PointCloud<pcl::PointXYZ>::Ptr kdtree_cloud;
		std::shared_ptr<pcl::KdTreeFLANN<pcl::PointXYZ>> kdtree_ptr;
				
	public:
	
	GraphMap(){
		kdtree_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
		kdtree_ptr.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());
		kdtree_ptr->setMinPts(1);
	}
	
	virtual ~GraphMap(){
		pointcloud_fingerprints.clear();
	}
	
	virtual bool add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud);
	
	virtual bool add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr, geometry_msgs::Pose);
};

} //mapper

#endif //_GRAPHMAP_H_
