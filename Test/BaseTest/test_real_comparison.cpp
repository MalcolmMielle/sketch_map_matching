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
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "vodigrex/linefollower/MultipleLineFollower.hpp"
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
	AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l_real;
	AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l_model;
	
	if(index == 0){
		mat_in = cv::imread("../Test/BaseTest/TEST_COMPARISON/TEST1/map/map.png");
		model = cv::imread("../Test/BaseTest/TEST_COMPARISON/TEST1/model/model.png");
	}
	
	else if(index == -1){
		
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
		mat_in = cv::imread("../Test/SLAM/Sequences/mapdrawn.png");
		model = cv::imread("../Test/SLAM/Sequences/mapdrawn2.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 2){
		mat_in = cv::imread("../Test/SLAM/Sequences/mapdrawn_test.png");
		model = cv::imread("../Test/SLAM/Sequences/mapdrawn_test-incomplete.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 3){
		mat_in = cv::imread("../Test/SLAM/Sequences/mapdrawn_test.png");
		model = cv::imread("../Test/SLAM/Sequences/mapdrawn_test-incomplete2.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 4){
		mat_in = cv::imread("../Test/SLAM/Sequences/mapdrawn_noloop.png");
		model = cv::imread("../Test/SLAM/Sequences/mapdrawn_incomplete_noloop.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 5){
		mat_in = cv::imread("../Test/SLAM/Sequences/mapdrawn_noloop.png");
		model = cv::imread("../Test/SLAM/Sequences/mapdrawn_incomplete_noloop2.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 6){
		mat_in = cv::imread("../Test/SLAM/Sequences/missingmap.png");
		model = cv::imread("../Test/SLAM/Sequences/fullmap.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 7){
		mat_in = cv::imread("../Test/SLAM/Sequences/halfmap.png");
		model = cv::imread("../Test/SLAM/Sequences/fullmap.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 8){
		mat_in = cv::imread("../Test/SLAM/Sequences/secondhalfmap.png");
		model = cv::imread("../Test/SLAM/Sequences/fullmap.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 9){
		mat_in = cv::imread("../Test/SLAM/Sequences/Sketchmapia/SM11_digit.png");
		model = cv::imread("../Test/SLAM/Sequences/Sketchmapia/SM11_digit_model.png");
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 10){
		mat_in = cv::imread("../Test/SLAM/Sequences/Remastered/0043.jpg");
		model = cv::imread("../Test/SLAM/Sequences/Remastered/0043_broken.jpg");
	
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}	
	else if(index == 11){
		mat_in = cv::imread("../Test/SLAM/Sequences/Remastered/0043_broken.jpg");
		model = cv::imread("../Test/SLAM/Sequences/Remastered/0043.jpg");
	
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	else if(index == 12){
		mat_in = cv::imread("../Test/SLAM/Sequences/Remastered/0043.jpg");
		model = cv::imread("../Test/SLAM/Sequences/Remastered/0043_long.jpg");
	
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}	
	else if(index == 13){
		mat_in = cv::imread("../Test/SLAM/Sequences/Remastered/0043_broken.jpg");
		model = cv::imread("../Test/SLAM/Sequences/Remastered/0043_long.jpg");
	
		
		cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
		cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	}
	//PREPROCESSING
	else if(index == 14){
		mat_in = cv::imread("../Test/BastTest/TEST_COMPARISON/TEST2/map/map.pgm");
		model = cv::imread("../Test/BaseTest/TEST_COMPARISON/TEST2/model/model.pgm");

		cv::threshold(mat_in, mat_in, 150, 255, CV_THRESH_BINARY);
		cv::threshold(model, model, 150, 255, CV_THRESH_BINARY);
		
		
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
		cv::erode(mat_in, mat_in, kernel);
		cv::erode(mat_in, mat_in, kernel);
		cv::erode(mat_in, mat_in, kernel);
		cv::erode(mat_in, mat_in, kernel);
		
		cv::erode(model, model, kernel);
		cv::erode(model, model, kernel);
		cv::erode(model, model, kernel);
		cv::erode(model, model, kernel);

		
		cv::subtract(cv::Scalar::all(255),mat_in,mat_in);
		cv::subtract(cv::Scalar::all(255),model,model);
		
		cv::floodFill(mat_in, cv::Point2i(0, 0), cv::Scalar(255));
		cv::floodFill(model, cv::Point2i(0 ,0), cv::Scalar(255));
		
		v_real.setLevel(25);
		v_model.setLevel(30);
		l_real.setMarge(30);
		l_model.setMarge(30);
		
	}
	
	else if(index == 15){
		mat_in = cv::imread("../Test/BaseTest/TEST_COMPARISON/TEST3/map/map.pgm");
		model = cv::imread("../Test/BaseTest/TEST_COMPARISON/TEST3/model/model.pgm");

// 		cv::threshold(mat_in, mat_in, 150, 255, CV_THRESH_BINARY);
// 		cv::threshold(model, model, 150, 255, CV_THRESH_BINARY);
// 		
		
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
		cv::erode(mat_in, mat_in, kernel);
		cv::erode(mat_in, mat_in, kernel);
		cv::erode(mat_in, mat_in, kernel);
		cv::erode(mat_in, mat_in, kernel);
		
		cv::erode(model, model, kernel);
		cv::erode(model, model, kernel);
		cv::erode(model, model, kernel);
		cv::erode(model, model, kernel);

		
		cv::subtract(cv::Scalar::all(255),mat_in,mat_in);
		cv::subtract(cv::Scalar::all(255),model,model);
		
		cv::floodFill(mat_in, cv::Point2i(0, 0), cv::Scalar(255));
		cv::floodFill(model, cv::Point2i(0 ,0), cv::Scalar(255));
		
		v_real.setLevel(30);
		v_model.setLevel(1);
		
	}
	
	else if(index == 16){
		mat_in = cv::imread("../Test/BaseTest/FromVirtual/Ravi/ObstacleMap.png");
		model = cv::imread("../Test/BaseTest/FromVirtual/map_2.png");
	}
	
	cv::imshow("map_in", mat_in);
	cv::imshow("model", model);
	
	cv::waitKey(0);
	
	
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
	
	cv::imshow("vlines", v_real_res);
	cv::imshow("clinemodel", v_model_res);
	
	cv::waitKey(0);
	
	
	
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
	
	cv::imshow("lines graph", draw_lines_graph);
	cv::imshow("line graph model", draw_lines_graph_model);
	
	cv::waitKey(0);
	
	AASS::graphmatch::PlaceExtractorList2Place place_real;
	AASS::graphmatch::PlaceExtractorList2Place place_model;
	place_real.inputMapIn(mat_in);
	gl_real.scale(2);
	place_real.inputGraph(gl_real);

	place_model.inputMapIn(model);
	gl_model.scale(2);
	place_model.inputGraph(gl_model);
	
	place_real.extract();
	place_model.extract();
	
	AASS::graphmatch::GraphPlace gp_real = place_real.getGraph();
	AASS::graphmatch::GraphPlace gp_model = place_model.getGraph();
	
	

	
	cv::Mat draw_pruned = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	cv::Mat draw_pruned_model = cv::Mat::zeros(model.size(), CV_8UC3);
	
	gp_real.drawSpecial(draw_pruned);
	gp_model.drawSpecial(draw_pruned_model);
	
	cv::imshow("place in", draw_pruned);
	cv::imshow("place model", draw_pruned_model);
	
	cv::waitKey(0);
	
	
	//MATCH
	
	AASS::graphmatch::GraphMatcherNeighbor graphmatch;
// 	AASS::graphmatch::GraphMatcherClusterFiltered graphmatchold;
	
	//MY THING
	
	graphmatch.planarEditDistanceAlgorithm(gp_real, gp_model);
	
// 	graphmatchold.planarEditDistanceAlgorithm(gp_real, gp_model);
	
	
	std::deque<	
			AASS::graphmatch::Hypothese
		> hypothesis_final = graphmatch.getResult();
	graphmatch.sort(hypothesis_final);
	
// 	std::deque<	
// 			AASS::graphmatch::Hypothese
// 		> hypothesis_final_old = graphmatchold.getResult();
// 	graphmatchold.sort(hypothesis_final_old);
		
		
	std::deque< 
		AASS::graphmatch::Match
		> list_result_old;
	
	int best =50;
// 	int best_old =50;
	
	//TODO : IDEA -> classify them by edit distance value and cluster the ones that are compatible => 0 or extremum vertex in common.
	int aa= 0;
	while(aa != -1){
		std::cout << "Which one ? " << std::endl;
		std::cin >> aa;
		best = hypothesis_final[aa].getDist();
	
// 		best_old = hypothesis_final_old[aa].getDist();
// 		list_result_old = hypothesis_final_old[aa].getMatches();
		
		hypothesis_final[aa].drawHypo(gp_real, gp_model, mat_in, model, "ALL FINAL", 1);
// 		graphmatchold.drawHypo(gp_real, gp_model, mat_in, model, list_result_old, "ALL FINAL OLD", 2);
		std::cout << "Distance is : " << best << std::endl;
// 		std::cout << "Distance old is : " << best_old << std::endl;
		cv::waitKey(10);
		
	}
	
}