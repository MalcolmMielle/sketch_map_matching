#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "GraphLine.hpp"
#include "vodigrex/voronoidiagram/ThinkerEVG.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "vodigrex/linefollower/MultipleLineFollower.hpp"



BOOST_AUTO_TEST_CASE(trying)
{
	
	std::cout << "SECOND TEST WITH GIMP IMAGE ******************************* /" << std::endl;
	AASS::vodigrex::ThinkerEVG tv;
	AASS::vodigrex::ThinkerVoronoi voro;
	cv::Mat mm = cv::imread("../Test/BaseTest/map_ssrr.png");
	cv::Mat invSrc =  cv::Scalar::all(255) - mm;
	tv.setPruning(false);
	tv.think(invSrc);

	
// 	std::cout << "Number of nodes" << gl.getNumVertices() << std::endl;
// 	
	cv::imshow("graph", tv.getResult());
	cv::imshow("origin", mm);
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
	
	
	voro.think(mm);
	cv::imshow("graph voro", voro.getResult());
	cv::imshow("origin", mm);
	cv::waitKey(0);
	
	
	
}