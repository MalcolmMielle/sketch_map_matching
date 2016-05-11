#ifndef VFLWRAPPER_GRAPH
#define VFLWRAPPER_GRAPH

#include <stdlib.h>     /* srand, rand */
#include "argraph.h"
#include "argedit.h"
#include "argloader.h"
#include <iostream>
#include <stdio.h>
#include "Place.hpp"
#include "bettergraph/HypotheseBase.hpp"
#include "bettergraph/MatchBase.hpp"
#include "GraphPlace.hpp"

namespace AASS{

	class PlaceDestroyer : public AttrDestroyer{
	public:
		virtual void destroy(void* p){
			delete p;
		}
	};


	class Empty{
		
	};

	std::istream& operator>>(std::istream& in, Empty &p){
		
		return in;
	}

	std::ostream& operator<<(std::ostream& in, Empty &p){
		
		return in;
	}

	
	class VFLGraph : public ARGraph<AASS::graphmatch::Place, Empty>{
		
	protected :
		
	public :
		
		VFLGraph(ARGLoader* loader) : ARGraph<AASS::graphmatch::Place, Empty>(loader){
			this-> SetNodeDestroyer(new PlaceDestroyer());
		};
	
		graphmatch::Place& getPlace(int i){ return *GetNodeAttr(i);}
		

		
		
	};
	
	
	class MatchVFL : public bettergraph::MatchBase<graphmatch::Place>{
	public :
		MatchVFL(const graphmatch::Place& v, const graphmatch::Place& vv) : 
			bettergraph::MatchBase<graphmatch::Place>(v, vv) 
			{};
			
		///@brief compare all attributes. return true if they are the same.
		virtual bool operator ==(const MatchVFL& m) const{
			if(_v1.mass_center == m.getFirst().mass_center && _v2.mass_center == m.getSecond().mass_center){
				return true;
			}
			return false;
		}
		
		
	};
	
	
	
	class HypotheseVFL : public bettergraph::HypotheseBase<MatchVFL>{
	public:
		HypotheseVFL(){};
		
		void extractPlace(std::vector<std::pair < int, int > > nodes, VFLGraph& input, VFLGraph& model){
			bettergraph::HypotheseBase<MatchVFL> hyp;
			for(size_t i = 0  ; i < nodes.size() ; ++i){
				MatchVFL match(input.getPlace(nodes[i].first), model.getPlace(nodes[i].second) );
				this->push_back(match);
			}
		}
		
		void drawHypo(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const string& name, bool invert, double scale) const;
		
		
		
	};
	
	inline void HypotheseVFL::drawHypo(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, bool invert, double scale) const 
	{
		std::cout << "INvert is " << invert << std::endl;
		cv::Mat obst_copy;
		obstacle.copyTo(obst_copy);
		
		cv::Mat obst_model_copy;
		obstacle_model.copyTo(obst_model_copy);
		
		cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
		cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
		cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
		
		int cols_max = obst_model_copy.size().width;
		if(cols_max < obst_copy.size().width){
			cols_max = obst_copy.size().width;
		}
		
		cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
		cv::Mat all = cv::Mat::zeros(size, CV_8UC3);
		cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
		cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);
		
		cv::Mat roi = all(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
		cv::Mat roi_linked = only_linked(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
		cv::Mat roi_model = all(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
		
		cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
		cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
		
		obst_copy.copyTo(roi_maps);
		obst_model_copy.copyTo(roi_model_maps);
		
		cv::Scalar color;
			
		if(draw_links.channels() == 1){
			color = 255;

		}
		
		else if(draw_links.channels() == 3){
			color[0] = 500;
			color[1] = 500;
			color[2] = 500;
		}
		
		cv::Scalar color_model;
			
		if(draw_links.channels() == 1){
			color_model = 100;
		}
		
		else if(draw_links.channels() == 3){
			color_model[1] = 100;
			color_model[2] = 100;
			color_model[3] = 100;
		}
		
		cv::Scalar color_one;
			
		if(draw_links.channels() == 1){
			color_one = 255;
		}
		
		else if(draw_links.channels() == 3){
			color_one[1] = 255;
			color_one[2] = 255;
			color_one[3] = 255;
		}
		
		cv::Scalar color_link_maps;
			
		if(all_maps.channels() == 1){
			color_link_maps = 255;
		}
		
		else if(all_maps.channels() == 3){
			color_link_maps[0] = 255;
			color_link_maps[1] = 255;
			color_link_maps[2] = 255;
		}
		
		//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
		gp_real.scale(scale);
		gp_model.scale(scale);
		
		
		gp_real.drawSpecial(roi);
		gp_model.drawSpecial(roi_model);
	
		
		for(size_t i = 0 ; i < _hypothesis.size() ; i++ ){
			
			
			cv::line(draw_links, _hypothesis[i].getFirst().mass_center, _hypothesis[i].getSecond().mass_center, color);
			cv::Point2i model_normal;
			cv::Point2i model;
			
			if(invert== false){
				//UGLY HACK BECAUSE THE SCALING DOESN'T WORK 
				model_normal = _hypothesis[i].getFirst().mass_center;
						
				model = _hypothesis[i].getSecond().mass_center;
				model.y = model.y + obst_copy.size().height;
			}
			else{

				//UGLY HACK BECAUSE THE SCALING DOESN'T WORK 
				
				model_normal = _hypothesis[i].getFirst().mass_center;
				model_normal.y = model_normal.y + obst_copy.size().height;
						
				model = _hypothesis[i].getSecond().mass_center;
			}
			
			cv::circle(draw_links, model, 10, color_model, 3);
			cv::circle(draw_links, model_normal, 10, color_one, 3);
			
			cv::line(all, model_normal, model, color, 5);
			
			cv::Scalar color_all_linked;
			cv::RNG rrng(12345);
			if(all.channels() == 1){
				color_all_linked = rrng.uniform(50, 255);
			}
			
			else if(all.channels() == 3){
				color_all_linked[1] = rrng.uniform(50, 255);
				color_all_linked[2] = rrng.uniform(50, 255);
				color_all_linked[3] = rrng.uniform(50, 255);
			}
			
			cv::circle(all, model, 10, color_all_linked, 3);
			cv::circle(all, model_normal, 10, color_all_linked, 3);
	
			//cv::line(all_maps, model_normal, model, color);
			cv::line(only_linked, model_normal, model, color);
			
			cv::circle(only_linked, model_normal, 10, color_one, -1);
			cv::line(all_maps, model_normal, model, color_link_maps, 5);
			cv::circle(all_maps, model_normal, 10, color, -1);
		
		
			cv::circle(only_linked, model, 10, color_model, -1);
			cv::line(all_maps, model_normal, model, color_link_maps, 5);
			cv::circle(all_maps, model, 10, color, -1);
			
			
		}
		
		
// 		gp_real.drawSpecial(draw_graph);
// 		gp_model.drawSpecial(draw_graph_model);
// 		
// 		gp_real.scale(1/scale);
// 		gp_model.scale(1/scale);
		
	// 	cv::imshow(name + "links", draw_links);
// 			cv::imshow("graph place", draw_graph);
// 			cv::imshow("model graph place", draw_graph_model);
		cv::imshow(name, all);
// 			cv::imshow(name + " maps", all_maps);
	// 	cv::imshow(name + " only linked", only_linked);
		
	}
	
	
	
	
}




#endif