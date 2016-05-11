#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "vodigrex/linefollower/LineFollowerDoors.hpp"
#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "PlaceExtractorList2Place.hpp"

#include "PlaceExtractorDoors.hpp"
#include "GlobalMatch.hpp"
#include "GraphMatcherAnchor.hpp"

#include "MapComparator/Util.hpp"

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

	AASS::graphmatch::VertexPlace vp;
	AASS::graphmatch::VertexPlace vp2;
	AASS::graphmatch::VertexPlace vp3;
	AASS::graphmatch::Hypothese hyp;
	AASS::graphmatch::Match m3(vp, vp2);
	AASS::graphmatch::Match m2(vp, vp2);
	
	hyp.push_back(m3);
	hyp.push_back(m2);
	
	std::cout << "Try now " << std::endl;
	
	try{
		AASS::graphmatch::containsDoubleMatch(hyp);
	}
	catch(const std::exception &e){
		std::cout << "This error is supopsed to happen : " << e.what() << std::endl;
	}
	
	hyp.clear();
	AASS::graphmatch::Match m4(vp3, vp2);
	hyp.push_back(m4);
	hyp.push_back(m3);
	
	try{
		AASS::graphmatch::containsDoubleLink(hyp);
	}
	catch(const std::exception &e){
		std::cout << "This error is supopsed to happen : " << e.what() << std::endl;
	}
	
	
	
		std::cout << 
	"/********************************************************************"
	<< std::endl << "second REAL test "<<std::endl;
	
	AASS::vodigrex::LineFollowerDoors<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> llll_3;
	AASS::vodigrex::LineFollowerDoors<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> lf_model;
	llll_3.setMarge(10);
	
	std::deque < cv::Point > dpoint;
// 	std::pair < cv::Point, cv::Point> pair;
// 	pair.first = cv::Point2i(100 ,0);
// 	pair.second = cv::Point2i(500 , 500);
	cv::Point pair, pair2;
	pair = cv::Point2i(100 ,500);
	pair2 = cv::Point2i(95 , 0);
	
// 	dpoint.push_back(pair);
// 	llll_3.setDoors(dpoint);
// 	llll_3.push_back(pair);
	llll_3.push_back(pair, pair2);
	lf_model.push_back(pair, pair2);


	cv::Mat bug = cv::imread("../Test/ObstacleMap.png");
	cv::Mat model = cv::imread("../Test/ObstacleMapplus.png");
	
	cv::imshow("model", model);
	
	AASS::vodigrex::ThinkerVoronoi t;
	AASS::graphmatch::PlaceExtractorList2Place p;
// 	Thinker_CGA t; 
	AASS::SketchMap m(bug.rows, bug.cols, t, p);
	m.setObstacleMat(bug);
	m.setMode(4);
	m.setDownSample(1);
	m.think();
	cv::Mat vlll_3 = m.getThinker()->getResult();
	
	AASS::vodigrex::ThinkerVoronoi t_model;
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
	cv::imshow("yoooo", vlll_3);
	cv::imshow("yoooo model", model_line);
	cv::waitKey(0);
	
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

	
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::imshow("graph model", maa_3_model);
	cv::waitKey(0);
	
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
	cv::imshow("GraphPlace model", draw_pruned_model );
	cv::waitKey(0);
	
	/* TESTING THE ANCHORS */
	
	AASS::graphmatch::GraphMatcherAnchor gma;
// 	gma.pushAnchor();
	std::deque < std::pair < AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > > anchors;
	
	getAnchor(gp, gp_model, anchors);
	BOOST_CHECK_EQUAL(anchors.size(), 2);
	
	for(size_t i = 0 ; i < anchors.size() ; i++){
		gma.pushAnchor(AASS::graphmatch::Match(anchors[i].first, anchors[i].second));
	}
	
	gma.match(gp, gp_model);
	
	std::deque<	
			AASS::graphmatch::Hypothese
		> hypothesis_final = gma.getResult();
		
	std::cout << "Size of final " << hypothesis_final.size() << std::endl;
	gma.sort(hypothesis_final);
	
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
		
		gma.drawHypo(gp, gp_model, bug, model, list_result, "ALL FINAL", 2);
		std::cout << "Distance is : " << best << std::endl;
		cv::waitKey(10);
		
	}
	
}