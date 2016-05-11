#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "SketchMap.hpp"
#include "Thinker_EVG.hpp"
#include "vodigrex/linefollower/MultipleLineFollower.hpp"



BOOST_AUTO_TEST_CASE(trying)
{
	
	std::cout << "SECOND TEST WITH GIMP IMAGE ******************************* /" << std::endl;
	AASS::Thinker_EVG tv;
	cv::Mat mm = cv::imread("../Test/BaseTest/ObstacleMapBug.png");
	tv.setPruning(false);
	tv.think(mm);
	

	
// 	std::cout << "Number of nodes" << gl.getNumVertices() << std::endl;
// 	
	cv::imshow("graph", tv.getResult());
	cv::waitKey(0);
	
	AASS::vodigrex::MultipleLineFollower<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> mlf;
	
	mlf.setD(2);
	mlf.inputMap(tv.getResult());
	mlf.thin();

	AASS::topologicalmap::GraphLine gl_real (mlf.getGraph(0));
	
	cv::Mat fin;
	fin = cv::Mat::zeros(mm.size().height, mm.size().width, CV_32F);
	gl_real.draw(fin);
	cv::imshow("final graph", fin);
	cv::waitKey(0);
	
	
}