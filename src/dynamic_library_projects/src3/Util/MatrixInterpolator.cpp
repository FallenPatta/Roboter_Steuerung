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

#include <Eigen/Geometry>

#include "Util/MatrixInterpolator.h"

namespace util{
	MatrixInterpolator::MatrixInterpolator(){
		
	}
	
	MatrixInterpolator::MatrixInterpolator(std::shared_ptr<std::deque<Eigen::Matrix4f>> start){
		this->start_matrices_ = start;
	}
	
	MatrixInterpolator::MatrixInterpolator(std::shared_ptr<std::deque<Eigen::Matrix4f>> start, std::shared_ptr<std::deque<Eigen::Matrix4f>> backward){
		this->start_matrices_ = start;
		this->backward_matrices = backward;
	}
	
	MatrixInterpolator::MatrixInterpolator(std::shared_ptr<std::deque<Eigen::Matrix4f>> start, std::shared_ptr<std::deque<Eigen::Matrix4f>> backward, std::shared_ptr<std::deque<Eigen::Matrix4f>> forward){
		this->start_matrices_ = start;
		this->backward_matrices = backward;
		this->forward_matrices = forward;
	}
	
	MatrixInterpolator::~MatrixInterpolator(){
		if(this->start_matrices_->size() > 0)
		{
			this->start_matrices_->clear();
		}
		
		if(this->backward_matrices->size() > 0)
		{
			this->backward_matrices->clear();
		}
		
		if(this->forward_matrices->size() > 0)
		{
			this->forward_matrices->clear();
		}
	}
	
	Eigen::Matrix4f MatrixInterpolator::invert_HMat(Eigen::Matrix4f a){
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
		
		return result;
	}


	Eigen::Matrix4f MatrixInterpolator::matrix_interpolate(Eigen::Matrix4f a, Eigen::Matrix4f b, double weight_a, double weight_b){
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
	
	void MatrixInterpolator::interpolate(Eigen::Matrix4f ref_pose, std::deque<Eigen::Matrix4f>& result_matrices, int iterations){
		
		std::deque<Eigen::Matrix4f> start_matrices = std::deque<Eigen::Matrix4f>(*start_matrices_);
		std::deque<Eigen::Matrix4f> tmp_matrices;
		Eigen::Matrix4f start_pose = start_matrices[0];
		for(int j = 0; j<iterations; j++)
		{
			
			//Backward pass
			tmp_matrices.clear();
			tmp_matrices.push_back(ref_pose);
			for(int i = 0; i<(*backward_matrices).size(); i++)
			{
				int from_mat = start_matrices.size()-1-i;
				Eigen::Matrix4f repositioned = (*backward_matrices)[i] * tmp_matrices[0];
				Eigen::Matrix4f interpolated = matrix_interpolate(repositioned, start_matrices[from_mat], (double)(*backward_matrices).size()-i, (double)i);
				//~ repositioned(0,3) = ((double)((*backward_matrices).size()-i)*repositioned(0,3) + (double)i*(start_matrices)[from_mat](0,3))/(double)(*backward_matrices).size();
				//~ repositioned(1,3) = ((double)((*backward_matrices).size()-i)*repositioned(1,3) + (double)i*(start_matrices)[from_mat](1,3))/(double)(*backward_matrices).size();
				//~ repositioned(2,3) = ((double)((*backward_matrices).size()-i)*repositioned(2,3) + (double)i*(start_matrices)[from_mat](2,3))/(double)(*backward_matrices).size();
				//~ tmp_matrices.push_front(repositioned);
				tmp_matrices.push_front(interpolated);
			}
			
			tmp_matrices.pop_back();
			start_matrices.clear();
			start_matrices.insert(start_matrices.begin(), tmp_matrices.begin(), tmp_matrices.end());
			
			//Forward pass
			tmp_matrices.clear();
			tmp_matrices.push_back(start_pose);
			for(int i = 0; i<(*forward_matrices).size(); i++)
			{
				Eigen::Matrix4f repositioned = (*forward_matrices)[i] * tmp_matrices.back();
				Eigen::Matrix4f interpolated = matrix_interpolate(repositioned, start_matrices[i+1], (double)(*forward_matrices).size()-i, (double)i);
				//~ repositioned(0,3) = ((double)((*forward_matrices).size()-i)*repositioned(0,3) + (double)i*(start_matrices)[i+1](0,3))/(double)(*forward_matrices).size();
				//~ repositioned(1,3) = ((double)((*forward_matrices).size()-i)*repositioned(1,3) + (double)i*(start_matrices)[i+1](1,3))/(double)(*forward_matrices).size();
				//~ repositioned(2,3) = ((double)((*forward_matrices).size()-i)*repositioned(2,3) + (double)i*(start_matrices)[i+1](2,3))/(double)(*forward_matrices).size();
				//~ tmp_matrices.push_back(repositioned);
				tmp_matrices.push_back(interpolated);
			}
			
			start_matrices.clear();
			start_matrices.insert(start_matrices.begin(), tmp_matrices.begin(), tmp_matrices.end());
		}
		
		result_matrices.insert(result_matrices.begin(), start_matrices.begin(), start_matrices.end());
	}
	
} //namespace util
