#ifndef UTILSPROBA_MALCOLM_GRAPH
#define UTILSPROBA_MALCOLM_GRAPH

#include "GraphProbaEdge.hpp"
#include "GraphMatcherNeighbor.hpp"
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include <iomanip>

namespace AASS{

	
	namespace probabilisticmatching{
		
		inline void printProbaGraph(const GraphProbaEdge& gpedge, const cv::Mat& in, const cv::Mat& in_ob)
		{
			std::cout << "Number of Num " << gpedge.getInput().getNumVertices() << std::endl;
			std::cout << "Number of Num model " << gpedge.getModel().getNumVertices() << std::endl;
			std::cout << "Print the PUUFF " << std::endl;
			gpedge.getPFFUUTable().print();
			std::cout << std::endl << std::endl;
			graphmatch::GraphPlace gp = gpedge.getInput();
			graphmatch::GraphPlace gp_model = gpedge.getModel();
			graphmatch::GraphMatcherNeighbor gm;
			int ju = 0 ;
			for(size_t i = 0 ; i < gpedge.size() ; ++i){
				graphmatch::Hypothese nodes;
				graphmatch::Match which = gpedge.getMatch(i);
				double best = which.getProba();
				
				nodes.push_back(which);
				
				
				std::cout << "Prioba " << std::setprecision(15) << best << " edge num " << gp.getNumEdges(which.getFirst()) << " " << gp_model.getNumEdges(which.getSecond()) << " " << i << "           ";
				if(ju % 3 == 0){
				
					std::cout << std::endl;
				}
				
				ju++;
				
				
				if(gp.getNumEdges(which.getFirst()) != gp_model.getNumEdges(which.getSecond()) && best < 0.682689 + 0.00001 && best > 0.682689 - 0.00001){
					std::cout << "FUCK MY LIFE at " << i << std::endl;
					
					nodes.drawHypo(gp, gp_model, in, in_ob, "proba", 1);
					cv::waitKey(0);
				}
				
			}
			

		}
		
		inline void printBestProbaGraph(const GraphProbaEdge& gpedge, const cv::Mat& in, const cv::Mat& in_ob)
		{
			graphmatch::GraphPlace gp = gpedge.getInput();
			graphmatch::GraphPlace gp_model = gpedge.getModel();
			graphmatch::GraphMatcherNeighbor gm;
			graphmatch::Match which = gpedge.getMatch(0);
			double best = which.getProba();
			for(size_t i = 1 ; i < gpedge.size() ; ++i){
				
				if(which.getFirst() != gpedge.getMatch(i).getFirst()){
					AASS::graphmatch::Hypothese nodes;
					nodes.push_back(which);
					nodes.drawHypo(gp, gp_model, in, in_ob, "proba");
					std::cout << "Prioba " << best << std::endl;
					cv::waitKey(0);
					best = -1;
					which = gpedge.getMatch(i);
				}
				
				else if(best < gpedge.getMatch(i).getProba()){
					best = gpedge.getMatch(i).getProba();
					which = gpedge.getMatch(i);
				}
				
			}
			

		}
		
		inline void printAlmostBestProbaGraph(const GraphProbaEdge& gpedge, const cv::Mat& in, const cv::Mat& in_ob)
		{
			graphmatch::GraphPlace gp = gpedge.getInput();
			graphmatch::GraphPlace gp_model = gpedge.getModel();
			graphmatch::GraphMatcherNeighbor gm;
			graphmatch::Match which = gpedge.getMatch(0);
			double best = which.getProba();
			for(size_t i = 1 ; i < gpedge.size() ; ++i){
				
				if(which.getFirst() != gpedge.getMatch(i).getFirst()){
					graphmatch::Hypothese nodes;
					nodes.push_back(which);
					nodes.drawHypo(gp, gp_model, in, in_ob, "proba");
					std::cout << "Proba chanre " << best << std::endl;
					cv::waitKey(0);
					best = -1;
					which = gpedge.getMatch(i);
				}
				
				else if(best < gpedge.getMatch(i).getProba()){
					best = gpedge.getMatch(i).getProba();
					which = gpedge.getMatch(i);
					graphmatch::Hypothese nodes;
					nodes.push_back(which);
					nodes.drawHypo(gp, gp_model, in, in_ob, "proba");
					
					cv::waitKey(0);
					best = -1;
					which = gpedge.getMatch(i);
				}
				
			}
			

		}

		
		
	}
}

#endif