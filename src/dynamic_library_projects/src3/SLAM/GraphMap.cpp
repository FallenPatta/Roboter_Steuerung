#include "SLAM/GraphMap.h"

#include <iostream>
#include <vector>
#include <deque>
#include <memory>
#include <random>

#include <stdlib.h>
#include <stdint.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace mapper{

	bool GraphMap::add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud){
		
		geometry_msgs::Pose alignment;
		
		alignment.position.x = 0;
		alignment.position.y = 0;
		alignment.position.z = 0;
		alignment.orientation.x = 0;
		alignment.orientation.y = 0;
		alignment.orientation.z = 0;
		alignment.orientation.w = 1.0;
		
		// Adding Pointclouds like this will not work - RAM will be filled within 1-2 Minutes
		// TODO: pointclouds_ should only store the "fingerprint" points of a cloud (local max Points of the scan)
		std::shared_ptr<std::pair<geometry_msgs::Pose, pcl::PointCloud<pcl::PointXYZ>::Ptr>> next_map;
		next_map.reset(new std::pair<geometry_msgs::Pose, pcl::PointCloud<pcl::PointXYZ>::Ptr>(alignment, input_cloud));
		
		//pointcloud_fingerprints.push_back(next_map);
		
		if(kdtree_cloud->size() == 0)
		{
			(*kdtree_cloud) += (*input_cloud);
			kdtree_ptr->setInputCloud(kdtree_cloud);
		}
		else
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<int> uniform_indices(1, (*input_cloud).size()-1);
			
			std::vector<int> indices_to_check;
			for(int i = 0; i<1024; i++)
			{
				indices_to_check.push_back(uniform_indices(gen));
			}
			
			if(kdtree_cloud->size() > 0)
			{
				pcl::PointCloud<pcl::PointXYZ> sparse_cloud;
				std::vector<int> nearest_points_indices;
				std::vector<float> nearest_points_sqrt_distances;
				int checkIndex = 0;
				for(int checkIndex : indices_to_check)
				{
					pcl::PointXYZ cloud_point = (*input_cloud).at(checkIndex);

					std::vector<int> nearest_point_index(1);
					std::vector<float> nearest_point_sqrt_distance(1);
					if(kdtree_ptr->nearestKSearch (cloud_point, 1, nearest_point_index, nearest_point_sqrt_distance) > 0)
					{
						if(nearest_point_index[0] >= 0 && nearest_point_index[0] < kdtree_cloud->size() && nearest_point_sqrt_distance[0] <= 0.01f)
						{
							nearest_points_indices.push_back(nearest_point_index[0]);
							sparse_cloud.push_back(cloud_point);
						}
					}
				}
				
				//~ std::cout << "indices: " << nearest_points_indices.size() << std::endl;
				//~ std::cout << "sparse cloud size: " << sparse_cloud.size() << std::endl;
				//~ std::cout << "KD size: " << kdtree_cloud->size() << std::endl;
				
				if(sparse_cloud.size() > 0 && kdtree_cloud->size() > 0){
					pcl::PointCloud<pcl::PointXYZ>::iterator iter = (*kdtree_cloud).begin();
					pcl::PointCloud<pcl::PointXYZ>::iterator sparse_cloud_iter = sparse_cloud.begin();
					int current_kdcloud_index = nearest_points_indices[0];
					for(int i = 0; i < nearest_points_indices.size(); i++)
					{
						current_kdcloud_index = nearest_points_indices[i];
						pcl::PointXYZ average;
						average.x = (sparse_cloud.at(i).x + (*kdtree_cloud).at(current_kdcloud_index).x)/2.0;
						average.y = (sparse_cloud.at(i).y + (*kdtree_cloud).at(current_kdcloud_index).y)/2.0;
						average.z = (sparse_cloud.at(i).z + (*kdtree_cloud).at(current_kdcloud_index).z)/2.0;
						kdtree_cloud->at(current_kdcloud_index) = average;
						++sparse_cloud_iter;
					}
					//~ kdtree_ptr->setInputCloud(kdtree_cloud);
				}
			}
		}
		return true;
	}
	
	bool GraphMap::add_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, geometry_msgs::Pose pose){
		
		std::shared_ptr<std::pair<geometry_msgs::Pose, pcl::PointCloud<pcl::PointXYZ>::Ptr>> next_map;
		next_map.reset(new std::pair<geometry_msgs::Pose, pcl::PointCloud<pcl::PointXYZ>::Ptr>(pose, input_cloud));
		
		pointcloud_fingerprints.push_back(next_map);
		
	}

} // mapper
