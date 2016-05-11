#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "vodigrex/linefollower/LineFollowerGraph.hpp"
#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "PlaceExtractorList2Place.hpp"
#include "GraphLine.hpp"


BOOST_AUTO_TEST_CASE(trying)
{
	
	/*AASS::topologicalmap::LineFollower lin;
	
	std::cout << 
	"/********************************************************************"
	<< std::endl << "first test "<<std::endl;
	
	cv::Mat map2 = cv::imread("../Test/simplemap.png", CV_LOAD_IMAGE_GRAYSCALE);
	map2.convertTo(map2, CV_8U);
	
	//cv::imshow("The map", map2);
	//cv::waitKey(0); 
	
	cv::Mat map = map2.clone();
	map.convertTo(map, CV_8U);
	
	lin.inputMap(map);
	
	//Build the first ROI
	lin.init();
	
	lin.findNextLPRP();
	cv::Point2i LP = lin.getLP();
	cv::Point2i RP = lin.getRP();
	
	std::cout << " LP" <<  LP << std::endl;
	std::cout << " RP" << RP << std::endl;
	
	BOOST_CHECK_EQUAL(3, LP.x);
	BOOST_CHECK_EQUAL(3, LP.y);
	BOOST_CHECK_EQUAL(4, RP.x);
	BOOST_CHECK_EQUAL(3, RP.y);
	lin.removeLineSegment(lin.getDynamicWindow());
	lin.moveForward();
	
	BOOST_CHECK_EQUAL(4, lin.getDynamicWindow().rows);
	BOOST_CHECK_EQUAL(5, lin.getDynamicWindow().cols);
	
	std::cout << lin.getDynamicWindow() << std::endl;
	
	lin.findNextLPRP();
	LP = lin.getLP();
	RP = lin.getRP();
	std::cout << " LP" <<  LP << std::endl;
	std::cout << " RP" << RP << std::endl;
	
	std::cout << lin.getDynamicWindow() << std::endl;
	lin.removeLineSegment(lin.getDynamicWindow());
	lin.moveForward();
	
	BOOST_CHECK_EQUAL(4, lin.getDynamicWindow().rows);
	BOOST_CHECK_EQUAL(5, lin.getDynamicWindow().cols);
	
	std::cout << lin.getDynamicWindow() << std::endl;
	
	while(lin.typeOfIntersection(lin.getDynamicWindow()) != -1){
		lin.findNextLPRP();
		lin.removeLineSegment(lin.getDynamicWindow());
		lin.moveForward();
		
		std::cout << std::endl;
		std::cout << lin.getDynamicWindow() << std::endl;
		
		std::cout << "type : " << lin.typeOfIntersection(lin.getDynamicWindow()) <<std::endl;
		
		cv::imshow("image ", map);
		cv::waitKey(30);
	}
	cv::waitKey(0);
	
	std::cout << 
	"/********************************************************************"
	<< std::endl << "Second test "<<std::endl;
	
	AASS::topologicalmap::LineFollower lin_way;
	
	cv::Mat way = cv::imread("../Test/way.png", CV_LOAD_IMAGE_GRAYSCALE);
	way.convertTo(way, CV_8U);
	//12 is perfect...
	lin_way.setD(2);
	lin_way.inputMap(way);
	
	//Build the first ROI
	lin_way.init();
	
	while(lin_way.typeOfIntersection(lin_way.getDynamicWindow()) != -1){
		lin_way.findNextLPRP();
		lin_way.removeLineSegment(lin_way.getDynamicWindow());
		lin_way.moveForward();
		cv::imshow("image 2 ", way);
		cv::waitKey(1);
		std::cout << std::endl;
		std::cout << lin_way.getDynamicWindow() << std::endl;
		
		std::cout << "type : " << lin_way.typeOfIntersection(lin_way.getDynamicWindow()) <<std::endl;
		
		cv::imshow("image ", lin_way.getResult());
		cv::waitKey(01);
	}
	
	cv::waitKey(0);*/
	
// 	std::cout << 
// 	"/********************************************************************"
// 	<< std::endl << "Third test "<<std::endl;
// 	
// 	AASS::topologicalmap::LineFollower linlin;
// 	
// 	cv::Mat wayway = cv::imread("../Test/way.png", CV_LOAD_IMAGE_GRAYSCALE);
// 	wayway.convertTo(wayway, CV_8U);
// 	
// 	linlin.setD(2);
// 	linlin.inputMap(wayway);
// 	linlin.init();
// 	
// 	linlin.thin();
// 	
// 	cv::imshow("image ", linlin.getResult());
// 	cv::waitKey(0);
// 
// 	std::cout << 
// 	"/********************************************************************"
// 	<< std::endl << "Fourth test "<<std::endl;
// 	
// 	AASS::topologicalmap::LineFollower l;
// 	
// 	cv::Mat mm = cv::imread("../Test/map2.png", CV_LOAD_IMAGE_GRAYSCALE);
// 	mm.convertTo(mm, CV_8U);
// 	
// 	l.setD(2);
// 	l.setMarge(0);
// 	l.inputMap(mm);
// 	l.thin();
// 	
// 	std::cout << "OUT OF ALL " << std::endl;
// // 	std::vector<AASS::topologicalmap::Intersection *> t = l.getIntersections();
// 	
// // 	for (int i =0 ; i < t.size() ; i++){
// // 		std::cout << "type of first intersection : " << t[i]->getType() << std::endl;
// // 	}
// 	cv::imshow("image ", l.getResult());
// 	cv::waitKey(0);
// 		std::cout << 
// 	"/********************************************************************"
// 	<< std::endl << "Fifth test "<<std::endl;
// 	
// 	AASS::topologicalmap::LineFollower ll;
// 
// 	cv::Mat vl = cv::imread("../Test/voronoiline.png", CV_LOAD_IMAGE_GRAYSCALE);
// 	vl.convertTo(vl, CV_8U);
// 
// 	ll.setD(2);
// 	ll.inputMap(vl);
// 	ll.thin();
// 	BOOST_CHECK_EQUAL(ll.getIntersections().size(), 2);
	
// 	AASS::topologicalmap::Intersection* m1 = ll.getIntersections()[0];
// 	AASS::topologicalmap::Intersection* m2 = ll.getIntersections()[1];
	
// 	std::vector<AASS::topologicalmap::Intersection *> tt = ll.getIntersections();
	
// 	for (int i =0 ; i < tt.size() ; i++){
// 		std::cout << "type of first intersection : " << tt[i]->getType() << std::endl;
// 	}
	
// 	ll.printGraph();
/*	
	cv::imshow("yoooo", ll.getResult());
	cv::waitKey(0);

		std::cout << 
	"/********************************************************************"
	<< std::endl << "Sixth test "<<std::endl;
	
	AASS::topologicalmap::LineFollower lll;

	cv::Mat vll = cv::imread("../Test/trail.png", CV_LOAD_IMAGE_GRAYSCALE);
	vll.convertTo(vll, CV_8U);

	lll.setD(2);
	lll.inputMap(vll);
	lll.thin();
	
	cv::imshow("yoooo", lll.getResult());
	cv::waitKey(0);
	
	
		std::cout << 
	"/********************************************************************"
	<< std::endl << "REAL test "<<std::endl;
	
	AASS::topologicalmap::LineFollower llll;

	cv::Mat vlll = cv::imread("../Test/vor_real.png", CV_LOAD_IMAGE_GRAYSCALE);
	vlll.convertTo(vlll, CV_8U);
	
	cv::imshow("yoooo", vlll);
	cv::waitKey(0);

	llll.setD(2);
	llll.inputMap(vlll);
	llll.thin();
	
// 	llll.printGraph();
	
	cv::Mat maa = vlll.clone();
	maa.setTo(cv::Scalar(0));
// 	llll.drawGraph(maa);
	
	cv::imshow("yoooo", llll.getResult());
	cv::imshow("graph", maa);
	cv::waitKey(0);
	*/
// 		std::cout << 
// 	"/********************************************************************"
// 	<< std::endl << "second REAL test "<<std::endl;
// 	
// 	AASS::topologicalmap::LineFollower llll_2;
// 
// 	cv::Mat vlll_2 = cv::imread("../Test/voronoiline.png", CV_LOAD_IMAGE_GRAYSCALE);
// 	vlll_2.convertTo(vlll_2, CV_8U);
// 	
// 	cv::imshow("yoooo", vlll_2);
// 	cv::waitKey(0);
// 
// 	llll_2.setD(2);
// 	llll_2.inputMap(vlll_2);
// 	llll_2.thin();
// 	
// 	llll_2.printGraph();
// 	
// 	cv::Mat maa_2 = vlll_2.clone();
// 	maa_2.setTo(cv::Scalar(0));
// 	llll_2.drawGraph(maa_2);
// 	
// 	cv::imshow("yoooo", llll_2.getResult());
// 	cv::imshow("graph", maa_2);
// 	cv::waitKey(0);
	
		std::cout << 
	"/********************************************************************"
	<< std::endl << "second REAL test "<<std::endl;
	
	AASS::vodigrex::LineFollowerGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> llll_3;
	llll_3.setMarge(10);

	cv::Mat bug = cv::imread("../Test/BaseTest/ObstacleMap.png");
	
	AASS::vodigrex::ThinkerVoronoi  t;
// 	Thinker_CGA t; 
	AASS::graphmatch::PlaceExtractorList2Place pe;
	AASS::SketchMap m(bug.rows, bug.cols, t, pe);
	
	m.setObstacleMat(bug);
	m.setMode(4);
	m.setDownSample(1);
	m.think();
	cv::Mat vlll_3 = m.getThinker()->getResult();
	
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);
	
// 	cv::Mat vlll_3;
// 	cv::cvtColor(vlll, vlll_3, CV_RGB2GRAY);
	vlll_3.convertTo(vlll_3, CV_8U);
	
	cv::imshow("yoooo", vlll_3);
	cv::waitKey(0);

	llll_3.setD(2);
	llll_3.inputMap(vlll_3);
	llll_3.thin();
	
	AASS::topologicalmap::GraphLine gl(llll_3.getGraph());
	gl.print();
	//g.fromCustom2Boost(llll_3.getIntersections());
	
	cv::Mat maa_3 = vlll_3.clone();
	maa_3.setTo(cv::Scalar(0));
	gl.draw(maa_3);
	
	cv::imshow("yoooo", llll_3.getResult());
	cv::imshow("graph", maa_3);
	cv::waitKey(0);
	
	llll_3.clear();
	
}