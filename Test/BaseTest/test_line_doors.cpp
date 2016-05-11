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


BOOST_AUTO_TEST_CASE(trying)
{

	
		std::cout << 
	"/********************************************************************"
	<< std::endl << "second REAL test "<<std::endl;
	
	AASS::vodigrex::LineFollowerDoors<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> llll_3;
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
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
// 	llll_3.push_back(pair);
	llll_3.push_back(pair2.first, pair2.second);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;

	cv::Mat bug = cv::imread("../Test/ObstacleMap.png");
	
	AASS::vodigrex::ThinkerVoronoi t;
	AASS::graphmatch::PlaceExtractorList2Place p;
// 	Thinker_CGA t; 
	AASS::SketchMap m(bug.rows, bug.cols, t, p);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	m.setObstacleMat(bug);
	m.setMode(4);
	m.setDownSample(1);
	m.think();
	cv::Mat vlll_3 = m.getThinker()->getResult();
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);
	
// 	cv::Mat vlll_3;
// 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
	vlll_3.convertTo(vlll_3, CV_8U);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);

	llll_3.setD(2);
	
	std::cout << "DOOR NUMBER " << llll_3.sizeDoor() << std::endl;
	llll_3.inputMap(vlll_3);
	
	std::cout << "DOOR NUMBER before thin " << llll_3.sizeDoor() << std::endl;
	llll_3.thin();
	
	AASS::topologicalmap::GraphLine gl(llll_3.getGraph());
	gl.print();
	//g.fromCustom2Boost(llll_3.getIntersections());
	
	cv::Mat maa_3 = vlll_3.clone();
	maa_3.setTo(cv::Scalar(0));
	gl.draw(maa_3);
// 	cv::line(maa_3, dpoint[0].first, dpoint[0].second, cv::Scalar(255));	

	
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::waitKey(0);
	
// 	llll_3.reset();
	
	
	
	AASS::graphmatch::PlaceExtractorDoors pd;
	
	pd.inputGraph(llll_3.getGraph());
	pd.extract();
	
	std::cout << "Getting the graph" << std::endl;
	AASS::graphmatch::GraphPlace gp = pd.getGraph(); 
	
	cv::Mat draw_pruned = cv::Mat::zeros(vlll_3.size(), CV_8UC3);
	std::cout << "Drawing " << gp.getNumVertices() << std::endl;
	gp.drawSpecial(draw_pruned );
	cv::imshow("GraphPlace", draw_pruned );
	cv::waitKey(0);
	
	/* TESTING THE ANCHORS */
	
	
	
	
	
	
	
}