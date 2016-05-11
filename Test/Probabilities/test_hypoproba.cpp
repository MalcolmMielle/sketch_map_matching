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
#include "Match.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	AASS::graphmatch::VertexPlace v;
	AASS::graphmatch::VertexPlace v2;
	
	AASS::graphmatch::Match m(v, v2);
	m.setProba(50);
	
	AASS::graphmatch::Hypothese H;
	H.push_back(m);
	
	BOOST_CHECK_EQUAL(50, H[0].getProba());
	
	
	
}