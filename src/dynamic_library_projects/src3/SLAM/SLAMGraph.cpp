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

	void SLAMGraph::removeNode(SLAMNode* address){
		for(std::vector<std::shared_ptr<SLAMNode*>>::iterator it = nodes.begin(); it != nodes.end(); ++it)
		{
			if(*(*it) == address)
			{
				nodes.erase(it);
			}
		}
	}
	
	std::vector<std::shared_ptr<SLAMNode*>> SLAMGraph::find_corresponding(SLAMNode* address){
		std::vector<std::shared_ptr<SLAMNode*>> results;
		for(std::vector<std::shared_ptr<SLAMNode*>>::iterator it = nodes.begin(); it != nodes.end(); ++it)
		{
			std::vector<cv::DMatch> matches;
			//~ node_matcher.match(address->, *descriptor_pair.second, matches);
		}
	}
	
} //namespace mapping

