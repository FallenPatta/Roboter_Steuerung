#include <SLAM/SLAMGraph.h>

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

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Geometry>

#include <SLAM/SLAMNode.h>
#include <SLAM/SLAMMatch.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_representation.h>

#include "Util/MatrixInterpolator.h"
#include "/home/david/Desktop/ros_workspace/src/dynamic_library_projects/src3/Util/PointCloudDisplay.hpp"

typedef pcl::PointXYZ PointType;

struct PointXYZ{
	float x,y,z,u;
};

int point_transform(PointXYZ* vector_array, const int array_size, float* mat_4x4);

namespace mapping{
	
	SLAMGraph::SLAMGraph() : pose_matrix(Eigen::Matrix4f::Identity()){
		scan_matcher.setTransformationEpsilon (0);
		scan_matcher.setMaxCorrespondenceDistance (0.05);
		scan_matcher.setMaximumIterations(7);
		scan_matcher.setEuclideanFitnessEpsilon (0);
		
		color_feature_generator = cv::ORB::create();
	}
	
	SLAMGraph::~SLAMGraph(){
		nodes.clear();
	}
	
	void SLAMGraph::clear(){
		nodes.clear();
	}

	int SLAMGraph::size(){
		return nodes.size();
	}

	void SLAMGraph::removeNode(std::shared_ptr<SLAMNode> address){
		for(std::vector<std::shared_ptr<SLAMNode>>::iterator it = nodes.begin(); it != nodes.end(); ++it)
		{
			if(*it == address)
			{
				nodes.erase(it);
			}
		}
	}
	
	void SLAMGraph::addNode(SLAMNode node){
		nodes.push_back(std::make_shared<SLAMNode>(node));
	}
	
	void SLAMGraph::addNode(std::shared_ptr<SLAMNode> address){
		nodes.push_back(address);
	}
	
	Eigen::Matrix4f SLAMGraph::lastPose(){
		return *nodes.back()->getPose();
	}
	
	std::shared_ptr<SLAMNode> SLAMGraph::getNode(int index){
		return nodes[index];
	}
	
	void SLAMGraph::setDisplayer(PointCloudDisplay * disp){
		displayer = disp;
	}
	
	/**
	 * Return: Vector of pairs
	 * first - id of node
	 * second - the node
	 */
	std::vector<Correspondence> SLAMGraph::find_corresponding(SLAMNode& node, int threshold, int num_correspondences, int after, bool backwards){
		
		std::vector<Correspondence> results;
		
		if((*node.getDescriptors()).rows <= threshold)
			return results;
		
		int id = 0;
		std::vector<std::shared_ptr<SLAMNode>>::iterator it;
		std::vector<std::shared_ptr<SLAMNode>>::iterator stop;
		if(backwards)
		{
			it = nodes.end()-(after+1);
			stop = nodes.begin()-1;
		}
		else
		{
			it = nodes.begin()+after;
			stop = nodes.end();
		}
		for(; it != stop; backwards ? --it : ++it)
		{
			
			if((*(**it).getDescriptors()).rows <= threshold)
				continue;
			
			std::vector<cv::DMatch> matches;
			node_matcher.match(*node.getDescriptors(), *(**it).getDescriptors(), matches);
			
			double mindist = 10000;
	
			for(cv::DMatch match : matches)
			{
				if(match.distance < mindist)
					mindist = match.distance;
			}
			
			if(mindist >= 100){
				continue;
			}
				
			std::vector<cv::DMatch> good_matches;
			
			for(cv::DMatch match : matches)
			{
				if(match.distance <= std::max(min_dist_relative_threshold * mindist, min_dist_absolute_threshold))
					good_matches.push_back(match);
			}
			
			if(good_matches.size() >= (int)threshold)
			{
				std::pair<int,std::shared_ptr<SLAMNode>> corr = std::pair<int,std::shared_ptr<SLAMNode>>(id,*it);
				int c_size = good_matches.size();
				Correspondence correspondence;
				correspondence.c = corr;
				correspondence.size = c_size;
				results.push_back(correspondence);
			}
			
			if(results.size() >= num_correspondences)
			{
				return results;
			}
			
			id++;
		}
		return results;
	}
	
	void SLAMGraph::transform_cloud_by_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix){
		float matrix[16] = {transform_matrix(0,0), transform_matrix(0,1), transform_matrix(0,2), transform_matrix(0,3), 
							transform_matrix(1,0), transform_matrix(1,1), transform_matrix(1,2), transform_matrix(1,3), 
							transform_matrix(2,0), transform_matrix(2,1), transform_matrix(2,2), transform_matrix(2,3), 
							transform_matrix(3,0), transform_matrix(3,1), transform_matrix(3,2), transform_matrix(3,3)};
	
		std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> * points = &cloud->points;
		PointXYZ * floated_points = (PointXYZ*)&((*points)[0]);
		::point_transform(floated_points, points->size(), matrix);
	}
	
	//~ void SLAMGraph::transform_cloud_by_matrix(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Eigen::Matrix4f transform_matrix){
		//~ std::cout << "used"  << transform_matrix << std::endl;
		//~ for(int i = 0; i<cloud->points.size(); i++){
			//~ Eigen::Vector4f p(0,0,0,0);
			//~ p(0) = cloud->points[i].x;
			//~ p(1) = cloud->points[i].y;
			//~ p(2) = cloud->points[i].z;
			//~ p(3) = 1.0;
			//~ p = transform_matrix * p;
			//~ cloud->points[i].x = p(0);
			//~ cloud->points[i].y = p(1);
			//~ cloud->points[i].z = p(2);
			
			//~ std::cout << "(" << cloud->points[i].x << "," << cloud->points[i].y << "," << cloud->points[i].z << ") --- ";
		//~ }
		//~ std::cout << "done" << std::endl;
	//~ }
	
	double SLAMGraph::calculateLoopError(int from, int to){
		double global_error = 0;
		
		for(int i = from; i<to-1; i++)
		{
			scan_matcher.setInputSource(nodes[i]->getCloud());
			scan_matcher.setInputTarget(nodes[i+1]->getCloud());
			global_error += scan_matcher.getFitnessScore();
		}
		
		scan_matcher.setInputSource(nodes[to]->getCloud());
		scan_matcher.setInputTarget(nodes[from]->getCloud());
		global_error += scan_matcher.getFitnessScore();
		
		return global_error;
	}
	
	Eigen::Matrix4f SLAMGraph::slerp(Eigen::Matrix4f src, Eigen::Matrix4f tar){
		Eigen::Matrix4f result;
		Eigen::Matrix3f in_src, in_tar, rot_result;
		Eigen::Quaternionf q_source, q_target, q_slerp;
		
		in_src(0,0) = src(0,0); in_src(1,0) = src(1,0); in_src(2,0) = src(2,0);
		in_src(0,1) = src(0,1); in_src(1,1) = src(1,1); in_src(2,1) = src(2,1);
		in_src(0,2) = src(0,2); in_src(1,2) = src(1,2); in_src(2,2) = src(2,2);
		
		in_tar(0,0) = tar(0,0); in_tar(1,0) = tar(1,0); in_tar(2,0) = tar(2,0);
		in_tar(0,1) = tar(0,1); in_tar(1,1) = tar(1,1); in_tar(2,1) = tar(2,1);
		in_tar(0,2) = tar(0,2); in_tar(1,2) = tar(1,2); in_tar(2,2) = tar(2,2);
		
		q_source = in_src;
		q_target = in_tar;
		q_slerp = q_source.slerp(0.5, q_target);
		rot_result = q_slerp;
		
		result(0,0) = rot_result(0,0); result(1,0) = rot_result(1,0); result(2,0) = rot_result(2,0);
		result(0,1) = rot_result(0,1); result(1,1) = rot_result(1,1); result(2,1) = rot_result(2,1);
		result(0,2) = rot_result(0,2); result(1,2) = rot_result(1,2); result(2,2) = rot_result(2,2);
		
		result(0,3) = (src(0,3)+tar(0,3))/2.0;
		result(1,3) = (src(1,3)+tar(1,3))/2.0;
		result(2,3) = (src(2,3)+tar(2,3))/2.0;
		result(3,3) = 1.0;
		
		result(3,0) = 0;
		result(3,1) = 0;
		result(3,2) = 0;
		
		return result;
	}
	
	//~ /**
	 //~ * Minimizes ICP Error between two indices of the Graphs scan list
	 //~ * 
	 //~ * closed_from: newer Node
	 //~ * closed_to: older Node 
	 //~ */
	//~ bool SLAMGraph::correct(int closed_from, int closed_to, Eigen::Matrix4f& final_pose){
				
		//~ if(closed_to >= closed_from)
		//~ {
			//~ std::cout << "ERROR: Wrong direction" << std::endl;
			//~ return false;
		//~ }
		
		//~ double error = 2.0;
		
		//~ //Cloud Poses -> [c0, c1, c2, c3, ... , c(to-from)]
		//~ std::shared_ptr<std::deque<Eigen::Matrix4f>> start_poses(new std::deque<Eigen::Matrix4f>());
		
		//~ std::shared_ptr<std::deque<Eigen::Matrix4f>> forwards_transforms(new std::deque<Eigen::Matrix4f>());
		
		//~ std::shared_ptr<std::deque<Eigen::Matrix4f>> backwards_transforms(new std::deque<Eigen::Matrix4f>());
		
		//~ //Init the cloud that will containt the (unused) output of icp
		//~ pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZ>);
		
		//~ //Store all Poses
		//~ for(int i = closed_to; i<=closed_from; i++)
		//~ {
			//~ (*start_poses).push_back(*nodes[i]->getPose());
		//~ }
		
		//~ // Get forward transforms
		//~ for(int i = closed_to; i<closed_from; i++)
		//~ {
			//~ //Init Nodes
			//~ std::shared_ptr<SLAMNode> node_A = nodes[i]; //Aligned to
			//~ std::shared_ptr<SLAMNode> node_B = nodes[i+1];  //Aligned
			
			//~ //Initialize Difference to Target Scan
			//~ Eigen::Matrix4f pose_B = *node_B->getPose();
			//~ Eigen::Matrix4f pose_A = *node_A->getPose();
			//~ Eigen::Matrix4f pose_diff = pose_A.inverse() * pose_B;
							
			//~ //Store for interpolation
			//~ forwards_transforms->push_back(pose_diff);
		//~ }
		
		
		//~ //##################################
		//~ // Important: Last Backward Transform is relation between the found correspondence scan and the newest scan
		//~ //##################################
		//~ //Init Nodes
		//~ std::shared_ptr<SLAMNode> node_A = nodes[closed_from]; //To-be-aligned
		//~ std::shared_ptr<SLAMNode> node_B = nodes[closed_to]; //Reference

		//~ //Initialize Difference to Target Scan
		//~ Eigen::Matrix4f pose_B = *node_B->getPose();
		//~ Eigen::Matrix4f pose_A = *node_A->getPose();
		//~ Eigen::Matrix4f pose_diff = pose_A.inverse() * pose_B;
				
		//~ //Transform Source Scan to Target Scan Pose
		//~ pcl::PointCloud<pcl::PointXYZ> transformed(node_A->getCloud()->width, node_A->getCloud()->height);
		//~ pcl::PointCloud<pcl::PointXYZ>::Ptr transf_shared = transformed.makeShared();
		//~ transform_cloud_by_matrix(node_A->getCloud(), transf_shared, pose_diff);

		//~ //Set input source to the new (to-correct) scan
		//~ scan_matcher.setInputSource(transf_shared);
		//~ scan_matcher.setInputTarget(node_B->getCloud());

		//~ //Perform Alignment
		//~ scan_matcher.align(*cloud_source_registered);

		//~ if(scan_matcher.getFitnessScore() > 0.5)
		//~ {
			//~ return false;
		//~ }

		//~ //Get the Transformation from c(i-1) to c(i)
		//~ Eigen::Matrix4f transform_matrix = scan_matcher.getFinalTransformation();

		//~ //THIS is the actual reference pose
		//~ Eigen::Matrix4f ref_pose = transform_matrix * pose_A;

		//~ //Store for interpolation
		//~ backwards_transforms->push_front(transform_matrix);
		//~ //##################################
		
		//~ // Get rest of backward transforms
		//~ for(int i = closed_from; i>closed_to; i--)
		//~ {
			//~ //Init Nodes
			//~ std::shared_ptr<SLAMNode> node_A = nodes[i-1]; //To-be-aligned
			//~ std::shared_ptr<SLAMNode> node_B = nodes[i]; //Reference
			
			//~ //Initialize Difference to Target Scan
			//~ Eigen::Matrix4f pose_B = *node_B->getPose();
			//~ Eigen::Matrix4f pose_A = *node_A->getPose();
			//~ Eigen::Matrix4f pose_diff = pose_A.inverse() * pose_B;
							
			//~ //Transform Source Scan to Target Scan Pose
			//~ pcl::PointCloud<pcl::PointXYZ> transformed(node_A->getCloud()->width, node_A->getCloud()->height);
			//~ pcl::PointCloud<pcl::PointXYZ>::Ptr transf_shared = transformed.makeShared();
			//~ transform_cloud_by_matrix(node_A->getCloud(), transf_shared, pose_diff);
			
			//~ //Set input source to the new (to-correct) scan
			//~ scan_matcher.setInputSource(transf_shared);
			//~ scan_matcher.setInputTarget(node_B->getCloud());
			
			//~ //Perform Alignment
			//~ scan_matcher.align(*cloud_source_registered);
			
			//~ if(scan_matcher.getFitnessScore() > 0.5)
			//~ {
				//~ return false;
			//~ }
			
			//~ //Get the Transformation from c(i-1) to c(i)
			//~ Eigen::Matrix4f transform_matrix = scan_matcher.getFinalTransformation();
			
			//~ //Store for interpolation
			//~ backwards_transforms->push_front(transform_matrix);
		//~ }
		
		//~ int iterations = 500;
		//~ std::deque<Eigen::Matrix4f> result;
		//~ util::MatrixInterpolator interpolator(start_poses, forwards_transforms, backwards_transforms);
		//~ interpolator.interpolate(ref_pose,result,iterations);
		
		//~ //Transform all clouds to their new positions
		//~ for(int i = closed_from+1; i<closed_to+1; i++)
		//~ {
			//~ Eigen::Matrix4f new_pose = result[i-closed_from];
			//~ Eigen::Matrix4f old_pose = *nodes[i]->getPose();
			//~ Eigen::Matrix4f transform = old_pose.inverse() * new_pose;
			
			//~ transform_cloud_by_matrix(nodes[i]->getCloud(), transform);
			//~ nodes[i]->setPose(std::make_shared<Eigen::Matrix4f>(new_pose));
		//~ }
		
		//~ final_pose = ref_pose;
		
		//~ return true;
	//~ }
	
	/**
	 * Calculates an (arbitrary) distance between two poses
	 */
	double SLAMGraph::getTransformDistance(Eigen::Matrix4f a, Eigen::Matrix4f b){
	
	Eigen::Vector3f charvec1;
	charvec1(0) = a(0,0) + a(0,1) + a(0,2);
	charvec1(1) = a(1,0) + a(1,1) + a(1,2);
	charvec1(2) = a(2,0) + a(2,1) + a(2,2);
	Eigen::Vector3f trans1;
	trans1(0) = a(0,3);
	trans1(1) = a(1,3);
	trans1(2) = a(2,3);
	//~ trans1 = trans1 / sqrt(trans1.norm());
	
	Eigen::Vector3f charvec2;
	charvec2(0) = b(0,0) + b(0,1) + b(0,2);
	charvec2(1) = b(1,0) + b(1,1) + b(1,2);
	charvec2(2) = b(2,0) + b(2,1) + b(2,2);
	Eigen::Vector3f trans2;
	trans2(0) = b(0,3);
	trans2(1) = b(1,3);
	trans2(2) = b(2,3);
	//~ trans2 = trans2 / sqrt(trans2.norm());
	
	double dist = pow(charvec1.x() - charvec2.x(),2) + pow(charvec1.y()- charvec2.y(),2) + pow(charvec1.z() - charvec2.z(),2);
	dist += pow(trans1.x() - trans2.x(),2) + pow(trans1.y() - trans2.y(),2) + pow(trans1.z() - trans2.z(),2);
	
	//~ dist = sqrt(dist);
	
	return dist;
}
	
void SLAMGraph::sparse_ification(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, int constant){
	int num = 0;
	for(pcl::PointXYZ point : *input)
	{
		num++;
		if(num % constant == 0)
		{
			if(!(point.x == 0 && point.y == 0 && point.z == 0))
				output->push_back(point);
			num += ( rand() % constant - (constant/2) );
		}
	}
}
	
	bool SLAMGraph::addFirstNode(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud, std::pair<ros::Time, std::shared_ptr<cv::Mat>> current_color_frame_){
		//Select random points
		pcl::PointCloud<pcl::PointXYZ>::Ptr sparse(new pcl::PointCloud<pcl::PointXYZ>);
		sparse_ification(current_pointcloud.second, sparse, 1000);
				
		//Create unit matrix
		Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
		
		//Generate Keypoints for current Cloud
		std::vector<cv::KeyPoint> keypoints;
		cv::Mat descriptors;
		color_feature_generator->detectAndCompute(*(current_color_frame_.second), cv::noArray(), keypoints, descriptors);
		
		//Convert descriptors
		if(descriptors.type() != CV_32F)
		{
			descriptors.convertTo(descriptors, CV_32F);
		}
		
		//Init SLAMNode for Graph
		std::shared_ptr<SLAMNode> newNode(new SLAMNode(
			sparse
			, transform_matrix
			, *(current_color_frame_.second)
			, keypoints
			, descriptors));
		
		//Add Node to Graph
		addNode(newNode);
		return true;
	}
	
	bool SLAMGraph::addNodeFromCloud(std::pair<ros::Time, pcl::PointCloud<pcl::PointXYZ>::Ptr> current_pointcloud, std::pair<ros::Time, std::shared_ptr<cv::Mat>> current_color_frame_){
		if(size() == 0){
			return addFirstNode(current_pointcloud, current_color_frame_);
		}
		else{
			//Generate input and target cloud(s) for icp
			pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input(new pcl::PointCloud<pcl::PointXYZ>);
			pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_target(new pcl::PointCloud<pcl::PointXYZ>);
			
			//sparseify last captured cloud
			sparse_ification(current_pointcloud.second, sparse_input, 1000); //TODO: Remove magic numbers
				
			displayer->publishCloud1(*current_pointcloud.second);
				
			//transform using GPU
			transform_cloud_by_matrix(sparse_input, pose_matrix); //TODO: FIX!

			//Generate Keypoints for current Cloud
			std::vector<cv::KeyPoint> keypoints;
			cv::Mat descriptors;
			color_feature_generator->detectAndCompute(*(current_color_frame_.second), cv::noArray(), keypoints, descriptors);
			
			//Convert descriptors
			if(descriptors.type() != CV_32F)
			{
				descriptors.convertTo(descriptors, CV_32F);
			}
			
			//Init SLAMNode with current (most likely wrong) pose
			std::shared_ptr<mapping::SLAMNode> newNode(new mapping::SLAMNode(
				sparse_input
				, pose_matrix
				, *(current_color_frame_.second)
				, keypoints
				, descriptors));
			
			//list of matching cloud candidates
			std::deque<std::shared_ptr<SLAMNode>> matching_candidates;


			// TODO: Magische Zahl (20) parametrisieren
			std::vector<Correspondence> corresponding_nodes = find_corresponding(*newNode, 20, cloud_candidate_backlog, 0, true);
			
			//~ std::cout << "Correspondences: (" << corresponding_nodes.size() << ")" <<std::endl;
			
			// store corresponding scans
			if(corresponding_nodes.size() > 0)
			{
				for(Correspondence c : corresponding_nodes)
				{
					matching_candidates.push_back(c.c.second);
				}
			}

			// If we are missing clouds: Add until we have enough scans or the graph is completely searched
			if(matching_candidates.size() <= cloud_candidate_backlog)
			{
				for(std::vector<std::shared_ptr<SLAMNode>>::reverse_iterator it = nodes.rbegin(); it != nodes.rend(); ++it)
				{
					std::shared_ptr<SLAMNode> n = *it;
					//enough?
					if(matching_candidates.size() == cloud_candidate_backlog)
					{
						break;
					}
					else
					{
						matching_candidates.push_back(*it);
					}
				}
			}
			
			//~ std::cout << "Candidates: (" << matching_candidates.size() << ")" <<std::endl;

			
			//Init the registered cloud (just to store the result - it's not actually used)
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_registered(new pcl::PointCloud<pcl::PointXYZ>);
			
			//Init a deque for matching neighbours
			std::deque<SLAMMatch> neighbours;
			
			//set the source as the last captured and now sparse cloud
			scan_matcher.setInputSource(newNode->getCloud());
			
			//Match for all candidates
			for(std::deque<std::shared_ptr<SLAMNode>>::reverse_iterator it = matching_candidates.rbegin(); it != matching_candidates.rend(); ++it)
			{
				//Set candidate as target
				scan_matcher.setInputTarget((*it)->getCloud());
				
				//Align source and target
				scan_matcher.align(*cloud_source_registered);
				
				//Check fit
				double fitness = scan_matcher.getFitnessScore(scan_matcher.getMaxCorrespondenceDistance());
				
				//If good enough -> add for later analysis
				if(fitness < mapping_epsilon && scan_matcher.hasConverged())
				{
					Eigen::Matrix4f transform_matrix = scan_matcher.getFinalTransformation();
					Eigen::Matrix4f transform_covariance_matrix = Eigen::Matrix4f::Identity();
					transform_covariance_matrix = fitness * transform_covariance_matrix;
					Eigen::Matrix4f rotation_covariance_matrix = Eigen::Matrix4f::Identity();
					rotation_covariance_matrix = fitness * rotation_covariance_matrix;
					neighbours.push_back(SLAMMatch(*it, std::make_shared<Eigen::Matrix4f>(transform_matrix), std::make_shared<Eigen::Matrix4f>(transform_covariance_matrix), std::make_shared<Eigen::Matrix4f>(rotation_covariance_matrix)));
				}
				
			}
			
			//~ std::cout << "Neighbours: (" << neighbours.size() << ")" <<std::endl;
			
			if(neighbours.size() > 0){
				
				displayer->display(*newNode->getImage(), *matching_candidates[0]->getImage(), *newNode->getDescriptors(), *matching_candidates[0]->getDescriptors(), *newNode->getKeypoints(), *matching_candidates[0]->getKeypoints());
				
				//~ std::cout << "GLS matching step" << std::endl;
				
				Eigen::Matrix4f newPose;
				//TODO: ADD Covarance for rotation
				Eigen::Matrix4f newCovariance;
				
				// use GLS to minimize local error
				generalized_least_square_estimation(neighbours, newPose, newCovariance);
								
				std::cout << "Transform: " << std::endl << newPose << std::endl;
				std::cout << "Covariance: " << std::endl << newCovariance << std::endl;
				
				//Set the final pose of the new Node
				newNode->setPose(std::make_shared<Eigen::Matrix4f>(newPose));
				//TODO: ADD Covarances
				pose_matrix = newPose;
				
				//Generate map addition from whole cloud
				pcl::PointCloud<pcl::PointXYZ>::Ptr sparse_input_storeable(new pcl::PointCloud<pcl::PointXYZ>);
				sparse_ification(current_pointcloud.second, sparse_input_storeable, 1000);
				
				//transform using GPU
				transform_cloud_by_matrix(sparse_input_storeable, pose_matrix);
				
				displayer->publishCloud2(*sparse_input_storeable);
				
				newNode->setCloud(sparse_input_storeable);
				
				//Add Node to Graph
				addNode(newNode);
				return true;
			}
			else{
				return false;
			}
			return false;
		}
		return false;
	}
	
	/**
	 * Performs the generalized least square estimation on the given matches and returns a pose matrix beta and pose covariance beta_covariance
	 * 
	 */
	 //TODO: Set beta_covarance to 2 3x3 Matrixes
	void SLAMGraph::generalized_least_square_estimation(std::deque<SLAMMatch> matches, Eigen::Matrix4f& beta, Eigen::Matrix4f& beta_covariance){
		int columns = 4 * matches.size();
		Eigen::MatrixXf X = Eigen::MatrixXf::Zero(4, columns);
		Eigen::MatrixXf y = Eigen::MatrixXf::Zero(4, columns);
		
		Eigen::MatrixXf Phi_inverse = Eigen::MatrixXf::Zero(columns, columns);
		
		for(int i = 0; i<matches.size(); i++){
			int start_index = 4*i;
			int end_index = 4*(i+1);
			y.block<4,4>(start_index, 0) = *(matches[i].getSourcePose());
			Eigen::Matrix4f tmp = (matches[i].getTransform())->inverse();
			X.block<4,4>(start_index, 0) = tmp;
			Eigen::Matrix4f diagonal = Eigen::Matrix4f::Identity();
			diagonal(0,0) = (matches[i].getTranslationCovariance())(0,0);
			diagonal(1,1) = (matches[i].getTranslationCovariance())(1,1);
			diagonal(2,2) = (matches[i].getTranslationCovariance())(2,2);
			diagonal(3,3) = (matches[i].getTranslationCovariance())(3,3);
			if(diagonal.isApprox(matches[i].getTranslationCovariance())){
				diagonal(0,0) = 1.0/(matches[i].getTranslationCovariance())(0,0);
				diagonal(1,1) = 1.0/(matches[i].getTranslationCovariance())(1,1);
				diagonal(2,2) = 1.0/(matches[i].getTranslationCovariance())(2,2);
				diagonal(3,3) = 1.0/(matches[i].getTranslationCovariance())(3,3);
				Phi_inverse.block<4,4>(start_index, start_index) = diagonal;
			}
			else{
				tmp = (matches[i].getTranslationCovariance()).inverse();
				Phi_inverse.block<4,4>(start_index, start_index) = tmp;
			}
		}
		
		Eigen::MatrixXf X_T = X.transpose();
		
		beta_covariance = (X_T * Phi_inverse * X).inverse();
		beta = beta_covariance * X_T * Phi_inverse * y;
	}
	
	/**
	 * If more than 10 transforms are to be taken into considderation, this function is used to split the estimation into smaller pieces and avoid the double-overflow problem
	 */
	void SLAMGraph::generalized_least_square_piecewise_estimation(std::deque<SLAMMatch> matches, Eigen::Matrix4f& beta, Eigen::Matrix4f& beta_covariance){
		if(matches.size() > 10){
			int pieces = std::ceil((double)matches.size()/10.0);
			std::deque<SLAMMatch> next_matches;
			for(int i = 0; i<pieces; i++){
				std::deque<SLAMMatch>::iterator start = matches.begin()+(i*(matches.size()/pieces));
				std::deque<SLAMMatch>::iterator end = matches.begin()+((i+1)*(matches.size()/pieces));
				if(i == pieces-1){
					end = matches.end();
				}
				std::deque<SLAMMatch> part_matches(start,end);
				
				Eigen::Matrix4f newPose;
				Eigen::Matrix4f newCovariance;
				
				generalized_least_square_estimation(part_matches, newPose, newCovariance);
				
				next_matches.push_back(SLAMMatch(std::make_shared<Eigen::Matrix4f>(newPose), std::make_shared<Eigen::Matrix4f>(Eigen::Matrix4f::Identity()), std::make_shared<Eigen::Matrix4f>(newCovariance)));
			}
			generalized_least_square_piecewise_estimation(next_matches, beta, beta_covariance);
		}
		else{
			generalized_least_square_estimation(matches, beta, beta_covariance);
		}
	}
} //namespace mapping











