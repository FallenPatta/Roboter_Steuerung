#include "SLAM/OctreeMapper.h"

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
//~ #include <pcl/octree/octree_nodes.h>

namespace mapper{

	bool OctreeMapper::add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
		octree_pointer->setInputCloud(input_cloud);
		octree_pointer->addPointsFromInputCloud();
		
		pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointXYZVector point_vector;
		octree_pointer->getOccupiedVoxelCenters(point_vector);
		
		//~ for(pcl::octree::OctreePointCloud<pcl::PointXYZ>::Iterator iter = octree_pointer->begin(); iter != octree_pointer->end(); ++iter){
			//~ if((*iter)->getNodeType() == pcl::octree::LEAF_NODE){
				//~ pcl::octree::OctreeLeafNode<pcl::PointXYZ>* node = static_cast<pcl::octree::OctreeLeafNode<pcl::PointXYZ>*>(*iter);
				//~ std::cout << "(" << (*node)->x << "," << (*node)->y << "," << (*node)->z << ")" << std::endl;
			//~ }
		//~ }
		
		//~ for(pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointXYZVector::iterator iter = point_vector.begin(); iter != point_vector.end(); ++iter){
			//~ std::cout << "(" << (*iter).x << "," << (*iter).y << "," << (*iter).z << ")" << std::endl;
		//~ }
		
		ROS_INFO("New Octree size: %lu", point_vector.size());
		
		return true;
	}
	
	
	/**
	 * TODO: Don't discard pose
	 */
	bool OctreeMapper::add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, geometry_msgs::Pose pose){
		
		octree_pointer->setInputCloud(input_cloud);
		octree_pointer->addPointsFromInputCloud();
		
		pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointXYZVector point_vector;
		octree_pointer->getOccupiedVoxelCenters(point_vector);
		
		ROS_INFO("New Octree size: %lu", point_vector.size());
		
		return true;
	}

} // mapper
