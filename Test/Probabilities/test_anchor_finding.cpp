#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "LineFollower.hpp"
#include "SketchMap.hpp"
#include "Thinker_Voronoi.hpp"
#include "PlaceExtractorList2Place.hpp"
#include "LineFollowerDoors.hpp"

#include "PlaceExtractorDoors.hpp"
#include "GlobalMatch.hpp"
#include "GraphMatcherAnchor.hpp"

#include "MapComparator/Util.hpp"

#include "ProbabilisticEditDistanceEdge.hpp"


void returnDoors(const AASS::graphmatch::GraphPlace& gp, std::deque < AASS::graphmatch::VertexPlace>& dvp){
	
	//first is beginning, second is "past the end"
	
	AASS::graphmatch::Door door;
	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
	//vertices access all the vertix
	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		if(door.isOfType(v, gp.getGraph())){
			dvp.push_back(v);
		}
		
	}
}



void getAnchor(const AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::GraphPlace& gp2, std::deque < std::pair < AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > >& dvp){
	
	std::deque < AASS::graphmatch::VertexPlace> dvp1;
	std::deque < AASS::graphmatch::VertexPlace> dvp2;
	
	returnDoors(gp, dvp1);
	returnDoors(gp2, dvp2);
	
	int size_gp = dvp1.size();
	int size_gp2 = dvp2.size();
	
	int min = 0;
	if(size_gp < size_gp2){
		min = size_gp;
	}
	else{
		min = size_gp2;
	}
	
	for(size_t i = 0 ; i < min ; i++){
		dvp.push_back(std::pair < AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace >(dvp1[i], dvp2[i]));
	}
	
}


BOOST_AUTO_TEST_CASE(trying)
{

	
		std::cout << 
	"/********************************************************************"
	<< std::endl << "second REAL test "<<std::endl;
	
	AASS::topologicalmap::LineFollowerDoors llll_3;
	AASS::topologicalmap::LineFollowerDoors lf_model;
	llll_3.setMarge(10);
	
	std::deque < std::pair < cv::Point, cv::Point > > dpoint;
	std::pair < cv::Point, cv::Point> pair;
	pair.first = cv::Point2i(100 ,0);
	pair.second = cv::Point2i(500 , 500);
	std::pair < cv::Point, cv::Point> pair2;
	pair2.first = cv::Point2i(100 ,500);
	pair2.second = cv::Point2i(100 , 0);
	
// 	dpoint.push_back(pair);
// 	llll_3.setDoors(dpoint);
// 	llll_3.push_back(pair);
	llll_3.push_back(pair2. first, pair2.second);
	lf_model.push_back(pair2.first, pair2.second);


	cv::Mat bug = cv::imread("../Test/ObstacleMap.png");
	cv::Mat model = cv::imread("../Test/ObstacleMapplus.png");
	
// 	cv::imshow("model", model);
	
	AASS::Thinker_Voronoi  t;
	AASS::graphmatch::PlaceExtractorList2Place p;
// 	Thinker_CGA t; 
	AASS::SketchMap m(bug.rows, bug.cols, t, p);
	m.setObstacleMat(bug);
	m.setMode(4);
	m.setDownSample(1);
	m.think();
	cv::Mat vlll_3 = m.getThinker()->getResult();
	
	AASS::Thinker_Voronoi  t_model;
	t_model.setLevel(25);
	AASS::graphmatch::PlaceExtractorList2Place p_model;
// 	Thinker_CGA t; 
	AASS::SketchMap sk_model(model.rows, model.cols, t_model, p_model);
	sk_model.setObstacleMat(model);
	sk_model.setMode(4);
	sk_model.setDownSample(1);
	sk_model.think();
	cv::Mat model_line = sk_model.getThinker()->getResult();
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
// 	cv::imshow("yoooo", vlll_3);
// 	cv::imshow("yoooo model", model_line);
// 	cv::waitKey(0);
	
// 	cv::Mat vlll_3;
// 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
	vlll_3.convertTo(vlll_3, CV_8U);
	model_line.convertTo(model_line, CV_8U);

	llll_3.setD(2);
	lf_model.setD(2);
	llll_3.inputMap(vlll_3);
	lf_model.inputMap(model_line);
	
	std::cout << "DOOR NUMBER before thin " << llll_3.sizeDoor() << std::endl;
	llll_3.thin();
	lf_model.thin();
	
	llll_3.printGraph();
	//g.fromCustom2Boost(llll_3.getIntersections());
	
	cv::Mat maa_3 = vlll_3.clone();
	maa_3.setTo(cv::Scalar(0));
	llll_3.drawGraph(maa_3);
	cv::Mat maa_3_model = model_line.clone();
	maa_3_model.setTo(cv::Scalar(0));
	lf_model.drawGraph(maa_3_model);
// 	cv::line(maa_3, dpoint[0].first, dpoint[0].second, cv::Scalar(255));	

	
// 	cv::imshow("yoooo", llll_3.getResult());
// 	cv::imshow("graph", maa_3);
// 	cv::imshow("graph model", maa_3_model);
// 	cv::waitKey(0);
	
// 	llll_3.reset();
	
	
	
	AASS::graphmatch::PlaceExtractorDoors pd;
	pd.inputGraph(llll_3.getGraph());
	pd.extract();
	
	AASS::graphmatch::PlaceExtractorDoors pd_model;
	pd_model.inputGraph(lf_model.getGraph());
	pd_model.extract();
	
	std::cout << "Getting the graph" << std::endl;
	AASS::graphmatch::GraphPlace gp = pd.getGraph(); 
	std::cout << "Getting the graph" << std::endl;
	AASS::graphmatch::GraphPlace gp_model = pd_model.getGraph(); 
	
	cv::Mat draw_pruned = cv::Mat::zeros(vlll_3.size(), CV_8UC3);
	gp.drawSpecial(draw_pruned );
	cv::imshow("GraphPlace", draw_pruned );
	cv::Mat draw_pruned_model = cv::Mat::zeros(vlll_3.size(), CV_8UC3);
	gp_model.drawSpecial(draw_pruned_model );
// 	cv::imshow("GraphPlace model", draw_pruned_model );
// 	cv::waitKey(0);
	
	/* TESTING THE ANCHORS */
	
	AASS::probabilisticmatching::ProbabilisticEditDistanceEdge ped;
	ped.init(gp, gp_model);
	std::deque < AASS::graphmatch::Match > anchor;
	
	ped.getAnchorsInit(gp, gp_model, anchor);
	
	BOOST_CHECK_EQUAL(2, anchor.size() );
	
	for(size_t i = 0 ; i < anchor.size() ; i++){
		std::cout << "types : " << gp[anchor[i].getFirst()].getID() << " " << gp_model[anchor[i].getFirst()].getID() << std::endl;
		std::cout << "Proba : " << anchor[i].getProba() << std::endl << std::endl;
	}
	
	ped.getAnchors(gp, gp_model, anchor);
	
	BOOST_CHECK_EQUAL(3, anchor.size() );
	
	for(size_t i = 0 ; i < anchor.size() ; i++){
		std::cout << "types : " << gp[anchor[i].getFirst()].getID() << " " << gp_model[anchor[i].getFirst()].getID() << std::endl;
		std::cout << "Proba : " << anchor[i].getProba() << std::endl << std::endl;
	}
	
	std::cout << "LAST" << std::endl;
	ped.getAnchors(gp, gp_model, anchor);
	
	BOOST_CHECK_EQUAL(4, anchor.size() );
	
	for(size_t i = 0 ; i < anchor.size() ; i++){
		std::cout << "types : " << gp[anchor[i].getFirst()].getID() << " " << anchor[i].getFirst() << " " << gp_model[anchor[i].getFirst()].getID() << " " << anchor[i].getSecond() << std::endl;
		std::cout << "Proba : " << anchor[i].getProba() << std::endl << std::endl;
	}
	
}