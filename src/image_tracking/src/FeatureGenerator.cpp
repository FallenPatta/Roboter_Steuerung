#include "FeatureGenerator.h"

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>      //for imshow

#include <vector>
#include <iostream>
#include <iomanip>

std::vector<cv::KeyPoint>* FeatureGenerator::get_keypoints(const cv::Mat& frame){
	std::vector<cv::KeyPoint>* keypoints = new std::vector<cv::KeyPoint>();
	
	detector_->detectAndCompute(frame, cv::noArray(), *keypoints, cv::noArray());
	
	return keypoints;
}
