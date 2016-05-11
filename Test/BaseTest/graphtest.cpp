#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "GraphList.hpp"


BOOST_AUTO_TEST_CASE(trying)
{
	topologicalmap::GraphList p; 
	std::string s = "yo";
	topologicalmap::Vertex u;
	p.addVertex(s, cv::Mat(), cv::Point2i(1, 1), u);
	topologicalmap::Vertex v;
	p.addVertex(s, cv::Mat(), cv::Point2i(), v, u);
	topologicalmap::Vertex k;
	p.addVertex(s, cv::Mat(), cv::Point2i(), k, v);
	
	BOOST_CHECK_EQUAL(3, p.getNumVertices() );
	p.print();
	
	if(u == u)
	{
		std::cout << "Sameeeee" << std::endl;
	}
	
	topologicalmap::Vertex vv;
	p.loopDetection(cv::Point2i(), u, vv);
	
	BOOST_CHECK_EQUAL(1, p.getGraph()[vv].point.x );

}