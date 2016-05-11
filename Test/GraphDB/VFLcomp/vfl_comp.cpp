#include "GraphDB.hpp"
#include "GraphPlace.hpp"


#include "Timed.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "ConversionVFL.hpp"
#include "match.h"
#include "vf2_sub_state.h"
#include "argloader.h"


#include "GraphMatcherClusterFiltered.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "GraphMatcherOld.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "MultipleLineFollower.hpp"
#include "PlaceExtractorList2Place.hpp"
#include "VFLVisitor.hpp"
#include "VFLwrapper.hpp"




bool my_visitor(int n, node_id ni1[], node_id ni2[], void *usr_data){
	
	std::cout << std::endl << "NEW MATCHING " << std::endl;
	for(int i = 0 ; i < n ; ++i) {
		std::cout << "Node " << ni1[i] << " is paired with node " << ni2[i] << std::endl;
	}
	return false;
}


AASS::graphmatch::Hypothese matchTest(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model){
	
	
	AASS::graphmatch::GraphMatcherNeighbor gm;
	
	std::ostringstream str_test;
	str_test <<  "test graphdb, line" << __LINE__ << " in file " << __FILE__;
	timed(str_test.str(), boost::bind( &AASS::graphmatch::GraphMatcherNeighbor::planarEditDistanceAlgorithm, &gm, boost::ref(gp), boost::ref(gp_model) ) );
	
	std::deque < AASS::graphmatch::Hypothese > res = gm.getResult();
	gm.sort(res);
	return res[0];
	
	
}


AASS::HypotheseVFL matchTest(AASS::VFLGraph& graph_vfl, AASS::VFLGraph& graph_vfl_model){
	
	//Graph subgraph isomorphisme using VF2
	VF2SubState s0_load(&graph_vfl, &graph_vfl_model);
	
	std::cout << std::endl << std::endl << "Second test " << std::endl;
	std::vector<std::pair < int, int > > vec;
	if(!match(&s0_load, AASS::visitorVector, &vec)){
		std::cout << "No matching found" << std::endl;
	}
	
	std::cout << "Input graph : smallest " << graph_vfl.NodeCount() << std::endl;
	std::cout << "model graph : biggest " << graph_vfl_model.NodeCount() << std::endl;
	
	AASS::HypotheseVFL vflhyp;
	vflhyp.extractPlace(vec, graph_vfl, graph_vfl_model);
	
	std::cout << "Matching size " << vflhyp.size() << " and vec size " << vec.size() << std::endl;
	
	return vflhyp;
	
}

int main(int argc,      // Number of strings in array argv
          char *argv[]   // Array of command-line argument strings
		 ){
	
	
	std::cout << "THE REAL THING" << std::endl;
	
	
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
	AASS::topologicalmap::MultipleLineFollower l_real;
	AASS::topologicalmap::MultipleLineFollower l_model;
	
	if(index == 0){
		mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
		model = cv::imread("../Test/TEST_COMPARISON/TEST1/model/model.png");
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
		mat_in = cv::imread("../Test/TEST_COMPARISON/TEST2/map/map.pgm");
		model = cv::imread("../Test/TEST_COMPARISON/TEST2/model/model.pgm");

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
		mat_in = cv::imread("../Test/TEST_COMPARISON/TEST3/map/map.pgm");
		model = cv::imread("../Test/TEST_COMPARISON/TEST3/model/model.pgm");

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
		mat_in = cv::imread("../Test/FromVirtual/Ravi/ObstacleMap.png");
		model = cv::imread("../Test/FromVirtual/map_2.png");
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
	
	AASS::topologicalmap::GraphList gl_real;
	gl_real = l_real.getGraph(0);
// 	gl_real.scale(2);
	AASS::topologicalmap::GraphList gl_model;
	gl_model = l_model.getGraph(0);
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
	
	
	/****************************************************************************
	 * Actual comparison is done after that
	 * 
	 * **************************************************************************/
										
	AASS::VFLGraph graph_vfl = AASS::graphPlace2VFL(gp_real);	
	AASS::VFLGraph graph_vfl_model = AASS::graphPlace2VFL(gp_model);
	
	std::cout << (*graph_vfl.GetNodeAttr(1)).mass_center << std::endl;;
	
	std::cout << "Size of input " << gp_real.getNumVertices() << std::endl;
	std::cout << "Size of model " << gp_model.getNumVertices() << std::endl;
	

	//Smallest graph needs to be first
	if(graph_vfl.NodeCount() < graph_vfl_model.NodeCount()){
		AASS::HypotheseVFL hyp_vfl = matchTest(graph_vfl, graph_vfl_model);
		hyp_vfl.drawHypo(gp_real, gp_model, mat_in, model, "VFL", false, 1);
	}
	else{
		AASS::HypotheseVFL hyp_vfl = matchTest(graph_vfl_model, graph_vfl);
		hyp_vfl.drawHypo(gp_real, gp_model, mat_in, model, "VFL", true, 1);
	}
	
	AASS::graphmatch::Hypothese hyp_mine = matchTest(gp_real, gp_model);	
	hyp_mine.drawMoved(gp_real, gp_model, mat_in, model, "Custom", 1);
	
	cv::waitKey(0);

	return 0;
}