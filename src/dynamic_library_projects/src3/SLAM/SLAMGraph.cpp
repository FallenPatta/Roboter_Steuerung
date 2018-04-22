#include <SLAM/SLAMGraph.h>

#include <iostream>
#include <vector>
#include <utility>
#include <deque>
#include <memory>

#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <SLAM/SLAMNode.h>

namespace mapping{
	
	SLAMGraph::SLAMGraph(){
		
	}
	
	SLAMGraph::~SLAMGraph(){
		nodes.clear();
	}
	
	void SLAMGraph::clear(){
		nodes.clear();
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
	
	void SLAMGraph::addNode(SLAMNode address){
		nodes.push_back(std::make_shared<SLAMNode>(address));
	}
	
	void SLAMGraph::addNode(std::shared_ptr<SLAMNode> address){
		nodes.push_back(address);
	}
	
	std::vector<std::shared_ptr<SLAMNode>> SLAMGraph::find_corresponding(SLAMNode& node, int threshold){
		
		std::vector<std::shared_ptr<SLAMNode>> results;
		
		if((*node.getDescriptors()).rows <= threshold)
			return results;
		
		for(std::vector<std::shared_ptr<SLAMNode>>::iterator it = nodes.begin(); it != nodes.end(); ++it)
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
				if(match.distance <= std::max(2*mindist, 0.05))
					good_matches.push_back(match);
			}
			
			if(good_matches.size() >= (int)threshold)
			{
				results.push_back(*it);
			}
		}
		return results;
	}
	
} //namespace mapping

