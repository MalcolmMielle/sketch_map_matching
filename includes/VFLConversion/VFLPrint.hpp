#ifndef VFLPRINT_GRAPH
#define VFLPRINT_GRAPH

#include <stdlib.h>     /* srand, rand */
#include "argraph.h"
#include "argedit.h"
#include "argloader.h"
#include <iostream>
#include <stdio.h>
#include "Place.hpp"

namespace AASS{

	inline scalePlace(std::vector<std::pair <graphmatch::Place, graphmatch::Place> >& vec_place, double scale){
		for(size_t i = 0 ; i < vec_place.size() ; ++i){
			vec_place[i].getFirst().mass_center * scale;
			vec_place[i].getSecond().mass_center * scale;
		}
	}
	
	
	inline void drawPlaces(std::vector<std::pair <graphmatch::Place, graphmatch::Place> >& vec_place, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const 
	{
		
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
		scalePlace(vec_place, scale);
		
		for(size_t i = 0 ; i < vec_place.size() ; i++ ){
			
			cv::line(draw_links, vec_place[i].getFirst().mass_center, vec_place[i].getSecond().mass_center, color);
			
			
			//UGLY HACK BECAUSE THE SCALING DOESN'T WORK 
			cv::Point2i model_normal;
			model_normal = vec_place[i].getFirst().mass_center;
					
			cv::Point2i model = vec_place[i].getSecond().mass_center;
			model.y = model.y + obst_copy.size().height;
			
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
		
		scalePlace(vec_place ,1/scale);
		
	// 	cv::imshow(name + "links", draw_links);
// 			cv::imshow("graph place", draw_graph);
// 			cv::imshow("model graph place", draw_graph_model);
		cv::imshow(name, all);
// 			cv::imshow(name + " maps", all_maps);
	// 	cv::imshow(name + " only linked", only_linked);
		
	}
	
	
}

#endif