#ifndef MATRIXINTERPOLATOR_H_
#define MATRIXINTERPOLATOR_H_

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

#include <Eigen/Geometry>

#include <omp.h>

namespace util{
	
	class MatrixInterpolator{
		public:
			MatrixInterpolator();
			MatrixInterpolator(std::shared_ptr<std::deque<Eigen::Matrix4f>> start);
			MatrixInterpolator(std::shared_ptr<std::deque<Eigen::Matrix4f>> start, std::shared_ptr<std::deque<Eigen::Matrix4f>> backward);
			MatrixInterpolator(std::shared_ptr<std::deque<Eigen::Matrix4f>> start, std::shared_ptr<std::deque<Eigen::Matrix4f>> backward, std::shared_ptr<std::deque<Eigen::Matrix4f>> forward);
			virtual ~MatrixInterpolator();
			
			void setInitial(std::shared_ptr<std::deque<Eigen::Matrix4f>> start);
			std::shared_ptr<std::deque<Eigen::Matrix4f>> getInitial();
			
			void setBackward(std::shared_ptr<std::deque<Eigen::Matrix4f>> backward);
			std::shared_ptr<std::deque<Eigen::Matrix4f>> getBackward();
			
			void setForward(std::shared_ptr<std::deque<Eigen::Matrix4f>> forward);
			std::shared_ptr<std::deque<Eigen::Matrix4f>> getForward();
			
			/**
			 * Runs the interpolation with forward and backward matrices.
			 * If there were any matrices in the initial result parameter they will be destroyed.
			 * 
			 * @param result The deque that will containt the resulting Transformations
			 * @param iterations The number of iterations the interpolation will be performed for
			 */
			void interpolate(Eigen::Matrix4f ref_pose, std::deque<Eigen::Matrix4f>& result, int iterations);
			
		protected:
			double min_delta_;
			std::shared_ptr<std::deque<Eigen::Matrix4f>> start_matrices_;
			std::shared_ptr<std::deque<Eigen::Matrix4f>> backward_matrices;
			std::shared_ptr<std::deque<Eigen::Matrix4f>> forward_matrices;
			
			Eigen::Matrix4f invert_HMat(Eigen::Matrix4f a);
			Eigen::Matrix4f matrix_interpolate(Eigen::Matrix4f a, Eigen::Matrix4f b, double weight_a, double weight_b);
	};
	
} //namespace util

#endif //MATRIXINTERPOLATOR_H_
