#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "GraphMatcherClusterFiltered.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "GraphMatcherOld.hpp"
#include "GraphMatcherAnchor.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "vodigrex/linefollower/MultipleLineFollower.hpp"
#include "PlaceExtractorRoomCorner.hpp"
#include "PlaceExtractorList2Place.hpp"

inline void test_cluster(std::deque< AASS::graphmatch::Hypothese >& hypothesis_final)
{

	//classify them in a clock wise manner
	std::deque<	
		AASS::graphmatch::Hypothese
	>::iterator hypothesis_final_ite;
	
	std::deque<	
		AASS::graphmatch::Hypothese
	>::iterator hypothesis_final_ite_2;
	
	AASS::graphmatch::Hypothese copy;
	
	
	for(hypothesis_final_ite = hypothesis_final.begin()+1 ; hypothesis_final_ite != hypothesis_final.end() ; hypothesis_final_ite++){
		
		hypothesis_final_ite_2 = hypothesis_final_ite ;
		copy = *hypothesis_final_ite;
		std::cout << "tsart" << std::endl;
		std::cout << "something" <<  ( * (hypothesis_final_ite_2 - 1) ).getDist() << " < " << (*hypothesis_final_ite_2).getDist()  << std::endl;
		while( hypothesis_final_ite_2 != hypothesis_final.begin() && (*(hypothesis_final_ite_2 - 1)).getDist() > copy.getDist()){
			
			std::cout << "MOVE" << std::endl;
			*( hypothesis_final_ite_2 ) = *( hypothesis_final_ite_2-1 );
			hypothesis_final_ite_2 = hypothesis_final_ite_2 - 1;
		
			
		}
		*(hypothesis_final_ite_2) = copy;
	}
	

}


BOOST_AUTO_TEST_CASE(trying)
{
	
	std::cout << "THE REAL THING" << std::endl;
	
	int argc = boost::unit_test::framework::master_test_suite().argc;
	char** argv = boost::unit_test::framework::master_test_suite().argv;
	int index;
	
	if(argc == 1){
		std::cout << "Need an argument number to know which model to load" << std::endl;
		
		std::cin >> index;
		
	}
	else{
		 index = atoi(argv[1]);
	}
	
	
	
	cv::Mat mat_in;
	cv::Mat model;
	AASS::vodigrex::ThinkerVoronoi v_real;
	AASS::vodigrex::ThinkerVoronoi v_model;
	AASS::graphmatch::PlaceExtractorRoomCorner prc;
	AASS::graphmatch::PlaceExtractorRoomCorner prc_model;
	AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l_real;
	AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l_model;
	
	if(index == -1){
		
		std::string adress;
		std::string adress_model;
		
		std::cout << "adress of map" << std::endl;
		std::cin >> adress;
		std::cout << "Model's address" << std::endl;
		std::cin >> adress_model;
		
		mat_in = cv::imread(adress);
		
		model = cv::imread(adress_model);
		
	}
	
	else if(index == 1){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Gael/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
		
// 		v_real.setLevel(34);
	}
	else if(index == 2){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Mom/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 3){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Chris/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
		
		v_real.setLevel(40);
	}
	else if(index == 4){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Han/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 5){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Ravi/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 6){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Maja/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 7){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Cori/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 8){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Mathieu/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 9){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Mathieu2/ObstacleMap.png");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
	}
	else if(index == 10){
		mat_in = cv::imread("../Test/Simulation/FromVirtual/Dorel/ObstacleMap.jpg");
		model = cv::imread("../Test/Simulation/FromVirtual/map_2.png");
		
		v_real.setLevel(15);
	}
	
	bool printing = true;
	
	if(printing == true){
		cv::imshow("map_in", mat_in);
		cv::imshow("model", model);
		
		cv::waitKey(0);
		
	}
	
	//PROCESSING OF PLACES
	
	v_real.modeLaplaceVoro();
	v_real.setDownSample(1);
	v_model.modeLaplaceVoro();
	v_model.setDownSample(1);
	
	v_real.think(mat_in);
	v_model.think(model);
	
	cv::Mat v_real_res =  v_real.getResult();
	cv::Mat v_model_res =  v_model.getResult();
	
// 	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
// 	cv::dilate(v_real_res, v_real_res, kernel);
// 	cv::dilate(v_model_res, v_model_res, kernel);
	
	if(printing == true){
		cv::imshow("vlines", v_real_res);
		cv::imshow("clinemodel", v_model_res);
		
		cv::waitKey(0);
	}
	
	if(printing == true){
		cv::imshow("map_in_again", mat_in);
		cv::imshow("model_again", model);
		
		cv::waitKey(0);
		
	}
	
	
	l_real.inputMap(v_real.getResult());
	l_real.thin();
	l_model.inputMap(v_model.getResult());
	l_model.thin();
	
	AASS::topologicalmap::GraphLine gl_real( l_real.getGraph(0) );
// 	gl_real.scale(2);
	AASS::topologicalmap::GraphLine gl_model( l_model.getGraph(0) );
// 	gl_model.scale(2);
	
	cv::Mat draw_lines_graph_model = cv::Mat::zeros(model.size(), CV_8UC3);
	cv::Mat draw_lines_graph = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	
	gl_real.draw(draw_lines_graph);
	gl_model.draw(draw_lines_graph_model);
	
	if(printing == true){
		cv::imshow("lines graph", draw_lines_graph);
		cv::imshow("line graph model", draw_lines_graph_model);
		
		cv::waitKey(0);
	}
	
	AASS::graphmatch::PlaceExtractorList2Place place_real;
	AASS::graphmatch::PlaceExtractorList2Place place_model;
	place_real.inputMapIn(mat_in);	
	gl_real.scale(2);
	place_real.inputGraph(gl_real);
// 	place_real.init();
	place_model.inputMapIn(model);
	gl_model.scale(2);
	place_model.inputGraph(gl_model);
// 	place_model.init();
	
// 	place_real.prunePreviousGraph();
// 	place_model.prunePreviousGraph();
	
	place_real.extract();
	place_model.extract();
	
	AASS::graphmatch::GraphPlace gp_real = place_real.getGraph();
	AASS::graphmatch::GraphPlace gp_model = place_model.getGraph();
	
// 	gl_model.scale(2);
// 	gl_real.scale(2);


// 	AASS::graphmatch::GraphPlace gp_model; 
// 	AASS::graphmatch::fromList2Place(gl_model, gp_model);
// 	
// 	AASS::graphmatch::GraphPlace gp_real; 
// 	AASS::graphmatch::fromList2Place(gl_real, gp_real);
// 	
// 	AASS::graphmatch::GraphPlace gp_model_old; 
// 	AASS::graphmatch::fromList2Place(gl_model, gp_model_old);
// 	
// 	AASS::graphmatch::GraphPlace gp_real_old; 
// 	AASS::graphmatch::fromList2Place(gl_real, gp_real_old);
	
	cv::Mat draw_pruned = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	cv::Mat draw_pruned_model = cv::Mat::zeros(model.size(), CV_8UC3);
	
	gp_real.drawSpecial(draw_pruned);
	gp_model.drawSpecial(draw_pruned_model);
	
	if(printing == true){
		cv::imshow("place in", draw_pruned);
		cv::imshow("place model", draw_pruned_model);
		
		cv::waitKey(0);
	}
	
	
	//MATCH
	
	AASS::graphmatch::GraphMatcherNeighbor graphmatch;
	AASS::graphmatch::GraphMatcherClusterFiltered graphmatchold;
	
	//MY THING
	
	graphmatch.planarEditDistanceAlgorithm(gp_real, gp_model);
	
	graphmatchold.planarEditDistanceAlgorithm(gp_real, gp_model);
	
	
	std::deque<	
			AASS::graphmatch::Hypothese
		> hypothesis_final = graphmatch.getResult();
	graphmatch.sort(hypothesis_final);
	
	std::deque<	
			AASS::graphmatch::Hypothese
		> hypothesis_final_old = graphmatchold.getResult();
	graphmatchold.sort(hypothesis_final_old);
		
		
	std::deque< 
		AASS::graphmatch::Match
		> list_result;
	std::deque< 
		AASS::graphmatch::Match
		> list_result_old;
	
	int best =50;
	int best_old =50;
	
	//TODO : IDEA -> classify them by edit distance value and cluster the ones that are compatible => 0 or extremum vertex in common.
	int aa= 0;
	while(aa != -1){
		std::cout << "Which one ? " << std::endl;
		std::cin >> aa;
		if(aa >= 0){
			best = hypothesis_final[aa].getDist();
			list_result = hypothesis_final[aa].getMatches();
			
			best_old = hypothesis_final_old[aa].getDist();
			list_result_old = hypothesis_final_old[aa].getMatches();
			
			if(printing == true){
// 				graphmatch.drawHypo(gp_real, gp_model, mat_in, model, list_result, "ALL FINAL", 2);
				hypothesis_final[aa].drawHypo(gp_real, gp_model, mat_in, model, "ALL FINAL no cale", 1);

				// 		graphmatchold.drawHypo(gp_real, gp_model, list_result_old, "ALL FINAL OLD", 2);
				std::cout << "Distance is : " << best << std::endl;
				std::cout << "Distance old is : " << best_old << std::endl;
				cv::waitKey(10);
			}
		}
		
	}
	
	
}