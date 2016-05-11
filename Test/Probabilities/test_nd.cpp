#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
//TODO : test with empty graph


#include "BaseProba.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	
	double stddev = 10;
	boost::math::normal_distribution<double> nd_test(2, stddev);
	boost::math::normal nd(2, stddev);
	//pdf stands for probibility density function
	//The probability density function is nonnegative everywhere, and its integral over the entire space is equal to one
	double var =  boost::math::pdf(nd, 2); 
	double var_test =  boost::math::pdf(nd_test, 2); 
	std::cout << var << std::endl;
	BOOST_CHECK(var <= 1);
	std::cout << var_test << std::endl;
	BOOST_CHECK(var_test <= 1);
	
	for(int i = 0 ; i < stddev*2 ; i++){
		var =  boost::math::cdf(nd, i + stddev); 
		var_test =  boost::math::cdf(nd, i - stddev);
		std::cout << " i : " << i << std::endl;
		std::cout << var << std::endl;
		BOOST_CHECK(var <= 1);
		std::cout << var - var_test << std::endl;
		BOOST_CHECK(var_test <= 1);
	}
	
	AASS::probabilisticmatching::BaseProba bp;
	
	for(int i = 0 ; i < stddev*2 ; i++){
		var = bp.normalDistribution(100, i , 5);
		std::cout << " i : " << i << std::endl;
		std::cout << var << std::endl;
	}
	
	
}