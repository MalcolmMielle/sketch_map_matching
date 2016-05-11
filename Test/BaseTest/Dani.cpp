#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "GraphMatcherClusterFiltered.hpp"
#include "Thinker_Voronoi.hpp"

#include "graph_b.h"
#include "graph_a.h"

std::deque < AASS::graphmatch::VertexPlace > vp_b;

float max_x = 0 ;
float max_y = 0;

float maxb_x = 0 ;
float maxb_y = 0;


void makeGraphA(AASS::graphmatch::GraphPlace& gp, int step){
	
	for(size_t i = 0 ; i < NVERT_A ; i=i+step){
		AASS::graphmatch::Place place;
		cv::Point2f pt(verts_a[i].x, verts_a[i].y);
		pt = pt*800;
		cv::Moments mo;
		std::vector <cv::Point> con;
		place.mass_center = pt;
		place.moment = mo;
		place.contour = con;
		AASS::graphmatch::VertexPlace vp_out;
		gp.addVertex(vp_out, place);
		vp_b.push_back(vp_out);
		std::cout << "value of point x " << pt.x << std::endl;
		if(pt.x > max_x){
			max_x = pt.x;
		}
		if(pt.y > max_y){
			max_y = pt.y;
		}
	}
	
	for(size_t i = 0 ; i < NCONN_A ; i=i+step){
	
		AASS::graphmatch::VertexPlace vp;
		vp = vp_b[connection_a[i].x];
		AASS::graphmatch::VertexPlace vp_2;
		vp_2 = vp_b[connection_a[i].y];
		AASS::graphmatch::Gateway g;
		AASS::graphmatch::EdgePlace out;
		gp.addEdge(vp, vp_2, out, g);
		
	}
	
	vp_b.clear();
	
}

void makeGraphB(AASS::graphmatch::GraphPlace& gp, int step){
	
	for(size_t i = 0 ; i < NVERT_B ; i=i+step){
		AASS::graphmatch::Place place;
		cv::Point2f pt(verts_b[i].x, verts_b[i].y);
		pt = pt*800;
		cv::Moments mo;
		std::vector <cv::Point> con;
		place.mass_center = pt;
		place.moment = mo;
		place.contour = con;
		place.mass_center = pt;
		AASS::graphmatch::VertexPlace vp_out;
		gp.addVertex(vp_out, place);
		vp_b.push_back(vp_out);
		std::cout << "value of point x " << pt.x << std::endl;
		if(pt.x > maxb_x){
			maxb_x = pt.x;
		}
		if(pt.y > maxb_y){
			maxb_y = pt.y;
		}
	}
	
	for(size_t i = 0 ; i < NCONN_B ; i=i+step){
	
		AASS::graphmatch::VertexPlace vp;
		vp = vp_b[connection_b[i].x];
		AASS::graphmatch::VertexPlace vp_2;
		vp_2 = vp_b[connection_b[i].y];
		AASS::graphmatch::Gateway g;
		AASS::graphmatch::EdgePlace out;
		gp.addEdge(vp, vp_2, out, g);
		
	}
	
}



BOOST_AUTO_TEST_CASE(trying)
{
	
	std::cout << "Dani Graph" << std::endl;
	
	cv::Mat mat_in;
	cv::Mat model;
	AASS::Thinker_Voronoi v_real;
	AASS::Thinker_Voronoi v_model;
	AASS::topologicalmap::LineFollower l_real;
	AASS::topologicalmap::LineFollower l_model;
	
	
	fill_b();
	fill_a();
	
	AASS::graphmatch::GraphPlace gp_a;
	AASS::graphmatch::GraphPlace gp_a2;
	
	makeGraphA(gp_a, 1);
	
	makeGraphA(gp_a2, 1);
	
	std::cout << "Max x " << max_x << " max y " << max_y << std::endl;
	cv::Mat draw = cv::Mat::zeros(cv::Size(max_x, max_y), CV_8UC3);
	gp_a.drawSpecial(draw);
	cv::imshow("Graph a", draw);
	cv::waitKey(0);
	
	AASS::graphmatch::GraphPlace gp_b;
	
	makeGraphB(gp_b, 1);
	std::cout << "Max x " << maxb_x << " max y " << maxb_y << std::endl;
	cv::Mat drawb = cv::Mat::zeros(cv::Size(maxb_x, maxb_y), CV_8UC3);
	gp_b.drawSpecial(drawb);
	cv::imshow("Graph b", drawb);
	cv::waitKey(0);

	
	cv::Mat mm = cv::Mat::zeros(cv::Size(max_x, max_y), CV_8UC3);

	AASS::graphmatch::GraphMatcherClusterFiltered graphmatch;
	
	//MY THING
	
	graphmatch.planarEditDistanceAlgorithm(gp_a, gp_b);
	
	std::deque<	
			AASS::graphmatch::Hypothese
		> hypothesis_final = graphmatch.getResult();
		
	if(hypothesis_final.size() == 0){
		std::cout << "No result" << std::endl;
		exit(0);
	}
		
	graphmatch.sort(hypothesis_final);
		
		
	std::deque< 
		AASS::graphmatch::Match
		> list_result;
	
	int best =50;
	
	//TODO : IDEA -> classify them by edit distance value and cluster the ones that are compatible => 0 or extremum vertex in common.
	int aa= 0;
	while(aa != -1){
		std::cout << "Which one ? " << std::endl;
		std::cin >> aa;
		best = hypothesis_final[aa].getDist();
		list_result = hypothesis_final[aa].getMatches();
		
		//OTHER PAPER
// 		std::deque<std::deque< AASS::graphmatch::Match > > H_other_paper;
// 		graphmatchold.init(gp_real, gp_model);
// 		graphmatchold.growHypotheses(gp_real, gp_model, H_other_paper);
		
// 		std::deque<	
// 				AASS::graphmatch::Hypothese
// 			> hypothesis_final_other;
// 			
// 		std::deque<std::deque< AASS::graphmatch::Match > >::iterator other_it;
// 		
// 		for(other_it = H_other_paper.begin() ; other_it != H_other_paper.end() ; other_it++){
// 			hypothesis_final_other.push_back(AASS::graphmatch::Hypothese(*other_it, (*other_it).size() ));
// 		}
		
// 		test_cluster(hypothesis_final_other);
// 		
// 		std::deque< 
// 			AASS::graphmatch::Match
// 			> list_result_other;
// 		
// 		list_result_other = hypothesis_final_other[hypothesis_final_other.size() - 1].getMatches();
		
		//PRINTING
		
		cv::Mat draw_links = cv::Mat::zeros(cv::Size(maxb_x, maxb_y), CV_8UC3);
		cv::Mat draw_graph = cv::Mat::zeros(cv::Size(max_x, max_y), CV_8UC3);
		cv::Mat draw_graph_model = cv::Mat::zeros(cv::Size(maxb_x, maxb_y), CV_8UC3);
		
// 		cv::Mat draw_links_other = cv::Mat::zeros(cv::Size(maxb_x, maxb_y), CV_8UC3);
// 		cv::Mat draw_graph_other = cv::Mat::zeros(cv::Size(max_x, max_y), CV_8UC3);
// 		cv::Mat draw_graph_model_other = cv::Mat::zeros(cv::Size(maxb_x, maxb_y), CV_8UC3);
		
		cv::Scalar color;
		cv::RNG rrng(12345);
			
		if(draw_links.channels() == 1){
			color = rrng.uniform(50, 255);
		}
		
		else if(draw_links.channels() == 3){
			color[1] = rrng.uniform(50, 255);
			color[2] = rrng.uniform(50, 255);
			color[3] = rrng.uniform(50, 255);
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
		
		
		for(size_t i = 0 ; i < list_result.size() ; i++ ){
			std::cout << "this : ";
			gp_a.print(list_result[i].getFirst());
			std::cout << " linked to this : " ;
			gp_b.print(list_result[i].getSecond());
			std::cout << std::endl;
			
			cv::line(draw_links, gp_a[list_result[i].getFirst()].mass_center, gp_b[list_result[i].getSecond()].mass_center, color);
			cv::circle(draw_links, gp_b[list_result[i].getSecond()].mass_center, 10, color_model, 3);
			cv::circle(draw_links, gp_a[list_result[i].getFirst()].mass_center, 10, color_one, 3);
			
		}
		
		gp_a.drawSpecial(draw_graph);
		gp_b.drawSpecial(draw_graph_model);
		
		cv::imshow("links at the end", draw_links);
		cv::waitKey(0);
		cv::imshow("graph place", draw_graph);
		cv::waitKey(0);
		cv::imshow("model graph place", draw_graph_model);
		cv::waitKey(0);
		
		//fuck up here
		graphmatch.drawHypo(gp_a, gp_b, mm, mm, list_result, "ALL FINAL");
		
// 		graphmatch.drawHypo(gp_real, gp_model, list_result_other, "ALL FINAL other paper");
		
		std::cout << "Distance is : " << best << std::endl;
		
		cv::waitKey(0);
	}
	
	
}