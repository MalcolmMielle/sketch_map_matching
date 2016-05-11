#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
//TODO : test with empty graph


#include "Hypothese.hpp"
#include "GraphProbaEdge.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	AASS::graphmatch::VertexPlace v;
	AASS::graphmatch::VertexPlace v2;
	
	AASS::graphmatch::Match m(v, v2);
	m.setProba(50);
	AASS::graphmatch::Match m2(v, v2);
	m2.setProba(25);
	AASS::graphmatch::Match m3(v, v2);
	m3.setProba(75);
	
	AASS::graphmatch::Hypothese H;
	H.push_back(m);
	
	BOOST_CHECK_EQUAL(50, H[0].getProba());
	
	std::vector < AASS::graphmatch::Match > match;
	match.push_back(m);
	match.push_back(m2);
	match.push_back(m3);
	
	AASS::probabilisticmatching::GraphProbaEdge en;;
	
	en.sort(match);
	
	
	for(size_t i = 0 ; i < match.size() ; i++){
	
		match[i].print();
		std::cout << std::endl;
	}
	
}