#ifndef FEATUREGENERATOR_H_
#define FEATUREGENERATOR_H_

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>      //for imshow

#include <vector>
#include <iostream>
#include <iomanip>

class FeatureGenerator{
	public:
	
	FeatureGenerator(cv::Ptr<cv::Feature2D> detector) : detector_(detector){};
	
	void get_keypoints(const cv::Mat& frame, std::vector<cv::KeyPoint>&, cv::Mat &);
	
	protected:
	
	cv::Ptr<cv::Feature2D> detector_;
    cv::Ptr<cv::Mat> current_frame_;
    std::vector<cv::KeyPoint> keypoints_;
};


#endif //FEATUREGENERATOR_H_
