#ifndef SLAMGRAPH_H_
#define SLAMGRAPH_H_

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
	
	class SLAMGraph{
		public:
			SLAMGraph();
			virtual ~SLAMGraph();
			void addNode(SLAMNode);
			void addNode(std::shared_ptr<SLAMNode>);
			void removeNode(std::shared_ptr<SLAMNode>);
			void clear();
			std::vector<std::shared_ptr<SLAMNode>> find_corresponding(SLAMNode&, int);
		protected:
			cv::FlannBasedMatcher node_matcher;
			std::vector<std::shared_ptr<SLAMNode>> nodes;
	};
	
} //namespace mapping

#endif //SLAMGRAPH_H_
