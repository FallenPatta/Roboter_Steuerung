#include "SLAM/FeatureGenerator.h"

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>      //for imshow

#include <vector>
#include <iostream>
#include <iomanip>

void FeatureGenerator::get_keypoints(const cv::Mat& frame, std::vector<cv::KeyPoint>& result, cv::Mat & descriptors){

	if(result.size() != 0)
	{
		result.clear();
	}
	
	detector_->detectAndCompute(frame, cv::noArray(), result, descriptors);
	
}
