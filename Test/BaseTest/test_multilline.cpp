#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "vodigrex/linefollower/MultipleLineFollower.hpp"
#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "GraphMatcherClusterFiltered.hpp"
#include "GraphMatcherNeighbor.hpp"
// #include "Global.hpp"
#include "PlaceExtractorList2Place.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	
	int argc = boost::unit_test::framework::master_test_suite().argc;
	char** argv = boost::unit_test::framework::master_test_suite().argv;
	bool extractplace;
	
	if(argc == 1){
		std::cout << "Need an argument number to know if we need to extract places or not" << std::endl;
		
		std::cin >> extractplace;
		
	}
	else{
		 extractplace = atoi(argv[1]);
	}
	
	if(extractplace == true){
		std::cout << "extrcting places" << std::endl;
	}
	else{
		std::cout << "No extracting places"<<std::endl;
	}
	
	/****Getting the model ***/
	AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> mlf_model;
	mlf_model.setMarge(10);

// 	cv::Mat bug = cv::imread("../Test/ObstacleMap.png");
	cv::Mat model = cv::imread("../Test/Sequences/Seq1/0052.jpg");
	cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
	
	cv::imshow("img2 original", model);
	cv::waitKey(0);
	
	cv::dilate(model, model, kernel);
	cv::dilate(model, model, kernel);	
	cv::erode(model, model, kernel);
	cv::erode(model, model, kernel);
	cv::erode(model, model, kernel);
	cv::erode(model, model, kernel);

	
	cv::imshow("img2", model);
	cv::waitKey(0);
	
	AASS::vodigrex::ThinkerVoronoi  t_model;
// 	t_model.setLevel(40);
// 	Thinker_CGA t; 
	AASS::graphmatch::PlaceExtractorList2Place pe;
	AASS::SketchMap m_model(model.rows, model.cols, t_model, pe);
	
	m_model.setObstacleMat(model);
	m_model.setMode(4);
	m_model.setDownSample(1);
	m_model.think();
	cv::Mat vlll_3 = m_model.getThinker()->getResult();
	
	cv::imshow("voronoi", vlll_3);
	cv::waitKey(0);
	vlll_3.convertTo(vlll_3, CV_8U);

	mlf_model.setD(2);
	mlf_model.inputMap(vlll_3);
	mlf_model.thin();
	
	AASS::topologicalmap::GraphLine gl_model ( mlf_model.getGraph(0) );
	AASS::graphmatch::GraphPlace gp_model;
	cv::Mat mat_l = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
	gl_model.draw(mat_l);
	cv::imshow("graph list model", mat_l);
	cv::waitKey(0);
	//Extracting places
	AASS::graphmatch::PlaceExtractorList2Place place_model;
	place_model.inputMapIn(model);
	place_model.inputGraph(gl_model);
	place_model.extract();
	gp_model = place_model.getGraph();
	
	cv::Mat mat_model = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
	gp_model.drawSpecial(mat_model);
	cv::imshow("graph model", mat_model);
	cv::waitKey(0);
	
	
	
	
	
	/*** Getting all the maps**/
	for(size_t i = 51 ; i > 0 ; i--){
		/** Getting the map **/
		AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> mlf;
		mlf.setMarge(10);

		std::string str;
		std::string name;
		if(i < 9){
			str = "../Test/Sequences/Seq1/000";
			name = "000";
		}
		else{
			str = "../Test/Sequences/Seq1/00";
			name = "00";
		}
		std::string str2;
		std::stringstream out;
		out << i + 1;
		str2 = out.str();
		
		str = str + str2 + ".jpg";
		name = name + str2;
		std::cout << "string : " << str << std::endl;;
		
		cv::Mat mreal = cv::imread(str);
		cv::threshold(mreal, mreal, 50, 255, CV_THRESH_BINARY_INV);
		
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
	
		cv::imshow("img3 original", mreal);
// 		cv::waitKey(0);
		
		cv::dilate(mreal, mreal, kernel);
		cv::dilate(mreal, mreal, kernel);	
		cv::erode(mreal, mreal, kernel);
		cv::erode(mreal, mreal, kernel);
		cv::erode(mreal, mreal, kernel);
		cv::erode(mreal, mreal, kernel);
		

			
		cv::imshow("img3", mreal);
		
		AASS::vodigrex::ThinkerVoronoi  t_real;
		AASS::graphmatch::PlaceExtractorList2Place pe;
	// 	Thinker_CGA t; 
		AASS::SketchMap map_real(mreal.rows, mreal.cols, t_real, pe);
		
		map_real.setObstacleMat(mreal);
		map_real.setMode(4);
		map_real.setDownSample(1);
		map_real.think();
		cv::Mat vlll_3_real = map_real.getThinker()->getResult();
		
		cv::imshow("voronoi_real", vlll_3_real);
		
		vlll_3_real.convertTo(vlll_3_real, CV_8U);

		mlf.setD(2);
		mlf.inputMap(vlll_3_real);
		mlf.thin();
		
		AASS::topologicalmap::GraphLine gl_real ( mlf.getGraph(0) ) ;

		cv::Mat mat_l_real = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
		gl_real.draw(mat_l_real);
		cv::imshow("graph real list", mat_l_real);
		AASS::graphmatch::GraphPlace gp_real;

		//Extracting places
		AASS::graphmatch::PlaceExtractorList2Place place_real;
		place_real.inputMapIn(mreal);
		place_real.inputGraph(gl_real);
		place_real.extract();
		gp_real = place_real.getGraph();

		cv::Mat mat_model_real = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
		gp_real.drawSpecial(mat_model_real);
		cv::imshow("graph model real", mat_model_real);

		
		/** Comparing **/
		AASS::graphmatch::GraphMatcherNeighbor graphmatch;
		std::cout << "Start of planar edit distance" << std::endl;

		graphmatch.planarEditDistanceAlgorithm(gp_real, gp_model);
		
		std::cout << "End of planar edit distance" << std::endl;
		
		std::deque<	
				AASS::graphmatch::Hypothese
			> hypothesis_final = graphmatch.getResult();
			
		std::deque< AASS::graphmatch::Match > list_result;
			
		if(hypothesis_final.size() > 0){
			
			graphmatch.sort(hypothesis_final);

			int best = hypothesis_final[0].getDist();
			list_result = hypothesis_final[0].getMatches();
			
// 			graphmatch.drawHypo(gp_real, gp_model, list_result, "ALL FINAL");
			graphmatch.drawExportData(gp_real, gp_model, mreal, model, list_result, name);
			std::cout << "Distance is : " << best << std::endl;
// 			cv::waitKey(0);
		
		}
		else{
			std::cout << "No match" << std::endl;
		}
		
	}
	
	
}