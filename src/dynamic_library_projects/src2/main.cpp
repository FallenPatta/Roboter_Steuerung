
#include <iostream>
#include <string>
#include <vector>
#include <utility>
#include <deque>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <memory>
#include <random>
#include <ctime>

#include <stdlib.h>
#include <stdint.h>

#include <omp.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "MessagePrinter.h"
#include "HelloWorldPrinter.h"
#include "GoodbyeWorldPrinter.h"
#include "PrinterComposite.h"
#include "DeallocatingPrinterComposite.h"

void broadcastTf0(std::deque<Eigen::Matrix4f> poses){
	static tf::TransformBroadcaster br;
	for(int i = 0; i<poses.size(); i++)
	{
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(poses[i](0,3), poses[i](1,3), poses[i](2,3)) );
		
		Eigen::Vector3f ea = poses[i].block<3,3>(0,0).eulerAngles(0, 1, 2);
		
		tf::Quaternion q;
		q.setRPY(ea.x(), ea.y(), ea.z());
		transform.setRotation(q);
		std::string name = "ref";
		name.append(std::to_string(i).c_str());
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name.c_str()));
	}
}

void broadcastTf1(std::deque<Eigen::Matrix4f> poses){
	static tf::TransformBroadcaster br;
	for(int i = 0; i<poses.size(); i++)
	{
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(poses[i](0,3), poses[i](1,3), poses[i](2,3)) );
		
		Eigen::Vector3f ea = poses[i].block<3,3>(0,0).eulerAngles(0, 1, 2);
		
		tf::Quaternion q;
		q.setRPY(ea.x(), ea.y(), ea.z());
		transform.setRotation(q);
		std::string name = "after";
		name.append(std::to_string(i).c_str());
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name.c_str()));
	}
}

void broadcastTf3(std::deque<Eigen::Matrix4f> poses){
	static tf::TransformBroadcaster br;
	for(int i = 0; i<poses.size(); i++)
	{
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(poses[i](0,3), poses[i](1,3), poses[i](2,3)) );
		
		Eigen::Vector3f ea = poses[i].block<3,3>(0,0).eulerAngles(0, 1, 2);
		
		tf::Quaternion q;
		q.setRPY(ea.x(), ea.y(), ea.z());
		transform.setRotation(q);
		std::string name = "opt";
		name.append(std::to_string(i).c_str());
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", name.c_str()));
	}
}

void broadcastTf2(std::deque<Eigen::Matrix4f> poses, Eigen::Matrix4f start){
	static tf::TransformBroadcaster br;
	
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(start(0,3), start(1,3), start(2,3)) );
	tf::Quaternion q;
	q.setRPY(0, 0, 0);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "-1"));
	
	for(int i = 0; i<poses.size(); i++)
	{
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(poses[i](0,3), poses[i](1,3), poses[i](2,3)) );
		
		Eigen::Vector3f ea = poses[i].block<3,3>(0,0).eulerAngles(0, 1, 2);
		
		tf::Quaternion q;
		q.setRPY(ea.x(), ea.y(), ea.z());
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), std::to_string(i-1).c_str(), std::to_string(i).c_str()));
	}
}

Eigen::Matrix4f invert_HMat(Eigen::Matrix4f a){
	Eigen::Matrix3f inv_rot = a.block(0,0,3,3).inverse();
	Eigen::Vector3f inv_trans;
	inv_trans(0) = a(0,3);
	inv_trans(1) = a(1,3);
	inv_trans(2) = a(2,3);
	inv_trans = (-1.0*inv_rot) * inv_trans;
	
	Eigen::Matrix4f result = Eigen::Matrix4f::Identity();
	result.block(0,0,3,3) = inv_rot;
	result(0,3) = inv_trans(0);
	result(1,3) = inv_trans(1);
	result(2,3) = inv_trans(2);
	
	//~ result(3,0) = 0;
	//~ result(3,1) = 0;
	//~ result(3,2) = 0;
	//~ result(3,3) = 1;
	return result;
}


Eigen::Matrix4f matrix_interpolate(Eigen::Matrix4f a, Eigen::Matrix4f b, double weight_a, double weight_b){
	Eigen::Matrix4f result = a;
	result(0,3) = (weight_a*a(0,3) + weight_b*b(0,3))/(weight_a + weight_b);
	result(1,3) = (weight_a*a(1,3) + weight_b*b(1,3))/(weight_a + weight_b);
	result(2,3) = (weight_a*a(2,3) + weight_b*b(2,3))/(weight_a + weight_b);
	
	double slerp_fraction = (weight_b) / (weight_a + weight_b);
	Eigen::Quaternionf rotation_a(a.block<3,3>(0,0));
	Eigen::Quaternionf rotation_b(b.block<3,3>(0,0));
	Eigen::Quaternionf rotation_result = rotation_a.slerp(slerp_fraction, rotation_b);
	
	Eigen::Matrix3f rot = rotation_result.toRotationMatrix();
	
	result(0,0) = rot(0,0);
	result(0,1) = rot(0,1);
	result(1,2) = rot(1,2);
	result(1,0) = rot(1,0);
	result(1,1) = rot(1,1);
	result(2,2) = rot(2,2);
	result(2,0) = rot(2,0);
	result(2,1) = rot(2,1);
	result(0,2) = rot(0,2);
	
	result(3,3) = 1;
	return result;
}

void interpolate(std::deque<Eigen::Matrix4f> start_matrices, std::deque<Eigen::Matrix4f> forward_matrices, std::deque<Eigen::Matrix4f> backward_matrices, Eigen::Matrix4f ref_pose, int iterations, std::deque<Eigen::Matrix4f>& result_matrices){
	std::deque<Eigen::Matrix4f> tmp_matrices;
	Eigen::Matrix4f start_pose = start_matrices[0];
	for(int j = 0; j<iterations; j++)
	{
		
		//Backward pass
		tmp_matrices.clear();
		tmp_matrices.push_back(ref_pose);
		for(int i = 0; i<backward_matrices.size(); i++)
		{
			int from_mat = start_matrices.size()-1-i;
			Eigen::Matrix4f repositioned = tmp_matrices[0] * backward_matrices[i]; //TODO: RICHTIGRUM MULTIPLIZIEREN
			Eigen::Matrix4f interpolated = matrix_interpolate(repositioned, start_matrices[from_mat], (double)backward_matrices.size()-i, (double)i);
			//~ repositioned(0,3) = ((double)(backward_matrices.size()-i)*repositioned(0,3) + (double)i*start_matrices[from_mat](0,3))/(double)backward_matrices.size();
			//~ repositioned(1,3) = ((double)(backward_matrices.size()-i)*repositioned(1,3) + (double)i*start_matrices[from_mat](1,3))/(double)backward_matrices.size();
			//~ repositioned(2,3) = ((double)(backward_matrices.size()-i)*repositioned(2,3) + (double)i*start_matrices[from_mat](2,3))/(double)backward_matrices.size();
			//~ tmp_matrices.push_front(repositioned);
			tmp_matrices.push_front(interpolated);
		}
		
		tmp_matrices.pop_back();
		start_matrices.clear();
		start_matrices.insert(start_matrices.begin(), tmp_matrices.begin(), tmp_matrices.end());
		
		//Forward pass
		tmp_matrices.clear();
		tmp_matrices.push_back(start_pose);
		for(int i = 0; i<forward_matrices.size(); i++)
		{
			Eigen::Matrix4f repositioned = tmp_matrices.back() * forward_matrices[i]; //TODO: RICHTIGRUM MULTIPLIZIEREN
			Eigen::Matrix4f interpolated = matrix_interpolate(repositioned, start_matrices[i+1], (double)forward_matrices.size()-i, (double)i);
			//~ repositioned(0,3) = ((double)(forward_matrices.size()-i)*repositioned(0,3) + (double)i*start_matrices[i+1](0,3))/(double)forward_matrices.size();
			//~ repositioned(1,3) = ((double)(forward_matrices.size()-i)*repositioned(1,3) + (double)i*start_matrices[i+1](1,3))/(double)forward_matrices.size();
			//~ repositioned(2,3) = ((double)(forward_matrices.size()-i)*repositioned(2,3) + (double)i*start_matrices[i+1](2,3))/(double)forward_matrices.size();
			//~ tmp_matrices.push_back(repositioned);
			tmp_matrices.push_back(interpolated);
		}
		
		start_matrices.clear();
		start_matrices.insert(start_matrices.begin(), tmp_matrices.begin(), tmp_matrices.end());
	}
	
	result_matrices.insert(result_matrices.begin(), start_matrices.begin(), start_matrices.end());
}

int main(int argc, char** argv){
	//~ print::MessagePrinter *printer1 = new print::HelloWorldPrinter();
	//~ print::MessagePrinter *printer2 = new print::GoodbyeWorldPrinter();
	//~ print::PrinterComposite * printerPointer = new print::DeallocatingPrinterComposite();
	//~ printerPointer->add(printer1);
	//~ printerPointer->add(printer2);
	//~ printerPointer->add(printer2);
	//~ printerPointer->add(printer2);	
	//~ std::cout << std::endl;
	//~ printerPointer->printMessage();
	
	//~ delete printerPointer;
	
	//~ return 0;
	
	ros::init(argc, argv, "tf_broadcaster");
	ros::NodeHandle node;
	
	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	
	double error = 0.1;
	double rad_error = 0.01;
	std::default_random_engine generator;
	//std::uniform_real_distribution<double> distribution1(-1*error,error); //forward
	//std::uniform_real_distribution<double> distribution2(-1*error,error); //backward
	
	std::normal_distribution<double> distribution1(0,error); //forward
	std::normal_distribution<double> distribution2(0,error); //backward
	
	std::normal_distribution<double> distribution3(0,rad_error); //angle

	ros::Rate rate(50);
	int iterations = 240;
	double v = 0;
	double step = 0.1;
	double true_step = 2.0*M_PI/((2.0*M_PI)/step);
	double x = cos(v);
	double y = sin(v);
	double z = sin(v);
	double radius = 3.0;
	while(ros::ok())
	{
		std::deque<Eigen::Matrix4f> start_matrices;
		std::deque<Eigen::Matrix4f> ground_truth_matrices;
		std::deque<Eigen::Matrix4f> forward_matrices;
		std::deque<Eigen::Matrix4f> backward_matrices;
		std::deque<Eigen::Matrix4f> tmp_matrices;
		
		std::deque<Eigen::Matrix4f> ref_matrices;
		
		//v+=0.02;
		
		for(int i = 0; i < (2.0*M_PI)/step; i++)
		{
			Eigen::Matrix4f m;
			
			Eigen::Matrix4f start_mat = Eigen::Matrix4f::Identity();
			x = radius*cos(v+step*(double)i);
			y = radius*sin(v+step*(double)i);
			z = radius*sin(3*(v+step*(double)i));
			start_mat(0,3) = x;
			start_mat(1,3) = y;
			start_mat(2,3) = z;
			m = Eigen::Affine3f(Eigen::AngleAxisf((double)i*step/2, Eigen::Vector3f::UnitZ())).matrix();
			m(3,3) = 1;
			//~ start_mat = m * start_mat;
			start_mat.block(0,0,3,3) = m.block(0,0,3,3);
								
			start_matrices.push_back(start_mat);
			
			Eigen::Matrix4f truth_mat = Eigen::Matrix4f::Identity();
			x = radius*cos(v+true_step*(double)i);
			y = radius*sin(v+true_step*(double)i);
			z = radius*sin(3*(v+true_step*(double)i));
			truth_mat(0,3) = x;
			truth_mat(1,3) = y;
			truth_mat(2,3) = z;
			m = Eigen::Affine3f(Eigen::AngleAxisf((double)i*true_step/2, Eigen::Vector3f::UnitZ())).matrix();
			m(3,3) = 1;
			//~ truth_mat = m * truth_mat;
			truth_mat.block(0,0,3,3) = m.block(0,0,3,3);
			
			ground_truth_matrices.push_back(truth_mat);
		}
		
		Eigen::Matrix4f ref_pose = ground_truth_matrices.back(); //Actually: Interpolation Goal Pose
		Eigen::Matrix4f start_pose = start_matrices[0]; //Actually: Interpolation Start Pose
		
		ground_truth_matrices.push_back(ground_truth_matrices.back());
		
		backward_matrices.push_front(Eigen::Matrix4f::Identity());
		
		for(int i = ground_truth_matrices.size()-2; i > 0; i--)
		{
			//~ Eigen::Matrix4f back_mat = ground_truth_matrices[i].inverse() * ground_truth_matrices[i-1];
			Eigen::Matrix4f back_mat = invert_HMat(ground_truth_matrices[i]) * ground_truth_matrices[i-1];

			back_mat(0,3) += distribution2(generator);
			back_mat(1,3) += distribution2(generator);
			back_mat(2,3) += distribution2(generator);
			
			Eigen::Matrix4f m;
			m = Eigen::Affine3f(Eigen::AngleAxisf(distribution3(generator), Eigen::Vector3f::UnitX())
									*Eigen::AngleAxisf(distribution3(generator), Eigen::Vector3f::UnitY())
									*Eigen::AngleAxisf(distribution3(generator), Eigen::Vector3f::UnitZ())).matrix();
			m(3,3) = 1;
			back_mat = m * back_mat;

			backward_matrices.push_back(back_mat);
		}
		
		ground_truth_matrices.pop_back();
		
		for(int i = 1; i < ground_truth_matrices.size(); i++)
		{
			//~ Eigen::Matrix4f for_mat = ground_truth_matrices[i-1].inverse() * ground_truth_matrices[i];
			Eigen::Matrix4f for_mat = invert_HMat(ground_truth_matrices[i-1]) * ground_truth_matrices[i];

			for_mat(0,3) += distribution1(generator);
			for_mat(1,3) += distribution1(generator);
			for_mat(2,3) += distribution1(generator);
			
			Eigen::Matrix4f m;
			m = Eigen::Affine3f(Eigen::AngleAxisf(distribution3(generator), Eigen::Vector3f::UnitX())
									*Eigen::AngleAxisf(distribution3(generator), Eigen::Vector3f::UnitY())
									*Eigen::AngleAxisf(distribution3(generator), Eigen::Vector3f::UnitZ())).matrix();
			m(3,3) = 1;
			for_mat = m * for_mat;

			forward_matrices.push_back(for_mat);
		}
		
		ref_matrices.push_back(ground_truth_matrices[0]);
		for(int i = 0; i < forward_matrices.size(); i++)
		{
			ref_matrices.push_back(ref_matrices.back() * forward_matrices[i]);
		}
		
		start_matrices.clear();
		start_matrices.insert(start_matrices.begin(), ref_matrices.begin(), ref_matrices.end());
		
		//~ int second_reference = 30;
		
		rate.sleep();
		//~ broadcastTf3(ground_truth_matrices);
		//~ broadcastTf0(ref_matrices);
		//~ broadcastTf1(start_matrices);
		
		std::deque<Eigen::Matrix4f> result_matrices;
		//###########################################
		for(int j = 0; j<iterations; j++)
		{
			result_matrices.clear();
			
			//~ std::deque<Eigen::Matrix4f> start_matrices_;
			//~ std::deque<Eigen::Matrix4f> forward_matrices_;
			//~ std::deque<Eigen::Matrix4f> backward_matrices_;
			//~ std::deque<Eigen::Matrix4f> result_matrices_;
			//~ Eigen::Matrix4f ref_pose_ = ground_truth_matrices[second_reference];
			//~ start_matrices_.insert(start_matrices_.begin(), start_matrices.begin(), start_matrices.begin()+second_reference);
			//~ forward_matrices_.insert(forward_matrices_.begin(), forward_matrices.begin(), forward_matrices.begin()+second_reference-1);
			//~ backward_matrices_.insert(backward_matrices_.begin(), backward_matrices.end()-second_reference, backward_matrices.end());
			
			//~ std::deque<Eigen::Matrix4f> start_matrices__;
			//~ std::deque<Eigen::Matrix4f> forward_matrices__;
			//~ std::deque<Eigen::Matrix4f> backward_matrices__;
			//~ std::deque<Eigen::Matrix4f> result_matrices__;
			//~ Eigen::Matrix4f ref_pose__ = ref_pose;
			//~ start_matrices__.insert(start_matrices__.begin(), start_matrices.begin()+second_reference, start_matrices.end());
			//~ forward_matrices__.insert(forward_matrices__.begin(), forward_matrices.begin()+second_reference-1, forward_matrices.end());
			//~ backward_matrices__.insert(backward_matrices__.begin(), backward_matrices.begin(), backward_matrices.end()-second_reference);
			
			//~ Eigen::Matrix4f reference_shift = start_matrices__.front().inverse() * ref_pose_;
			//~ for(int k = 0; k<start_matrices__.size(); k++)
			//~ {
				//~ start_matrices__[k] = reference_shift * start_matrices__[k];
			//~ }
			
			//~ interpolate(start_matrices_, forward_matrices_, backward_matrices_, ref_pose_, 1, result_matrices_);
			//~ interpolate(start_matrices__, forward_matrices__, backward_matrices__, ref_pose__, 1, result_matrices__);
			
			//~ result_matrices.clear();
			//~ result_matrices.insert(result_matrices.begin(), result_matrices_.begin(), result_matrices_.end());
			//~ result_matrices.insert(result_matrices.end(), result_matrices__.begin(), result_matrices__.end());
			
			interpolate(start_matrices, forward_matrices, backward_matrices, ref_pose, 1, result_matrices);
			
			start_matrices.clear();
			start_matrices.insert(start_matrices.begin(), result_matrices.begin(), result_matrices.end());
						
			//Visualize
			broadcastTf3(ground_truth_matrices);
			broadcastTf0(ref_matrices);
			broadcastTf1(start_matrices);
			
			//~ broadcastTf2(forward_matrices, ground_truth_matrices[0]);
			//~ broadcastTf2(backward_matrices, ground_truth_matrices.back());
			
			rate.sleep();
		}
		//###########################################
		
	}
	return 0;
	
}
