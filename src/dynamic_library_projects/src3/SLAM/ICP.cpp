#include "SLAM/ICP.h"
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

	bool ICP::localize(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, geometry_msgs::PoseWithCovariance estimate, geometry_msgs::PoseWithCovariance& result, bool continuous){
		return false;
	}

} // localizer
