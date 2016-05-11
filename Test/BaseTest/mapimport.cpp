#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>

#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "PlaceExtractorList2Place.hpp"
BOOST_AUTO_TEST_CASE(trying)
{
	AASS::vodigrex::ThinkerVoronoi tv;
	AASS::graphmatch::PlaceExtractorList2Place pe;
	AASS::SketchMap m(tv, pe);
	m.clear();
	
	m.addObstacle(0,1, true);
	m.addObstacle(2,3, false);
	
	m.addPath(22, 23, true);
	
	m.addDoors(cv::Point2i(0, 15), cv::Point2i(16,17));
	
	AASS::Robot r(42, 24);
	r.setX(42);
	r.setY(24);
	
	std::cout << "well : "<<r.getX()<<std::endl;
	
	m.addRobot(r);
	m.exportSystem2File();
	
	
	//Import file again
	AASS::SketchMap m2(tv, pe);
	
	m2.setPathData("Export");
	m2.importFile2System();
	
	BOOST_CHECK_EQUAL(0,m2.getObstacles().at(0).x );
	BOOST_CHECK_EQUAL(1,m2.getObstacles().at(0).y );
	BOOST_CHECK_EQUAL(2,m2.getObstacles().at(1).x );
	BOOST_CHECK_EQUAL(3,m2.getObstacles().at(1).y );
	
	BOOST_CHECK_EQUAL(22,m2.getPath().at(0).x );
	BOOST_CHECK_EQUAL(23,m2.getPath().at(0).y );
	
	//std::cout << "SOO WHAT : "<<m2.getRobot().size()<< " " <<m2.getRobot().at(0).getX()<<std::endl;
	
	BOOST_CHECK_EQUAL(42,m2.getRobots().at(0).getX() );
	BOOST_CHECK_EQUAL(24,m2.getRobots().at(0).getY() );
	
	BOOST_CHECK_EQUAL(m2.getDoors().at(0).x, 0);
	BOOST_CHECK_EQUAL(m2.getDoors().at(0).y, 15);
	BOOST_CHECK_EQUAL(m2.getDoors().at(1).x, 16);
	BOOST_CHECK_EQUAL(m2.getDoors().at(1).y, 17);
	
	
	/******************** Test add obstacles special****************/
	
	m.addObstacle(15,14, true);
	m.addObstacle(500,500, false);
	
	cv::Mat show= cv::Mat::ones(500, 500, CV_8UC3);
	
	for(int i=0; i< m.getObstacles().size(); i++){
		cv::circle(show, cv::Point2d(m.getObstacles()[i].x, m.getObstacles()[i].x), 5, cv::Scalar(255,0,255), -1);
	
	}
	
	std::string name("hoy");
	cv::namedWindow(name, 1);
	cv::imshow(name, show);
	
	cv::waitKey(0);
	
}