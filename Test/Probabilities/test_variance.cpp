#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
//TODO : test with empty graph

#ifndef UNIT_TEST
#define UNIT_TEST
#endif

#include "VarianceEdgeNumber.hpp"
#include "GraphProbaEdge.hpp"



namespace unit_test {
// Static wrappers for private/protected methods
struct VarianceTester
{
  static void setNumberSimilar(AASS::probabilisticmatching::VarianceEdgeNumber& foo, AASS::graphmatch::GraphPlace& gp)
  {
    foo.setNumberSimilar(gp);
  }
  
  static const std::vector<double>& getFactors(const AASS::probabilisticmatching::VarianceEdgeNumber& foo)
  {
    return foo.getFactors();
  }
  
};
}



AASS::graphmatch::VertexPlace v1;
AASS::graphmatch::VertexPlace v2;
AASS::graphmatch::VertexPlace v3;
AASS::graphmatch::VertexPlace v4;
AASS::graphmatch::VertexPlace v5;
AASS::graphmatch::VertexPlace v6;

AASS::graphmatch::VertexPlace g2_v1;
AASS::graphmatch::VertexPlace g2_v2;
AASS::graphmatch::VertexPlace g2_v3;

AASS::graphmatch::VertexPlace g2_v4;

//TODO : test with empty graph


void createGraph(AASS::graphmatch::GraphPlace& gp_test_editdistance){

	
	AASS::graphmatch::VertexPlace vnew_test_editdistance;
	AASS::graphmatch::VertexPlace v2new_test_editdistance;
	AASS::graphmatch::VertexPlace v1new_test_editdistance;
	AASS::graphmatch::VertexPlace v3new_test_editdistance;
	AASS::graphmatch::VertexPlace v4new_test_editdistance;
	AASS::graphmatch::VertexPlace v5new_test_editdistance;
	
	std::vector <cv::Point> conte_test_editdistance;
	cv::Point2i mcnew_test_editdistance(400, 400); 
	cv::Point2i mcnew1_test_editdistance(100, 2); //r    1
	cv::Point2i mcnew2_test_editdistance(150, 5); //c    4
	cv::Point2i mcnew3_test_editdistance(7, 100); //r    2
	cv::Point2i mcnew4_test_editdistance(3, 150); //r    3
	cv::Point2i mcnew5_test_editdistance(4, 400); //c    5
	
	cv::Moments m_test_editdistance;
	
	AASS::graphmatch::Place place;
	place.contour = conte_test_editdistance;
	place.moment = m_test_editdistance;
	place.mass_center = mcnew_test_editdistance;
	
	gp_test_editdistance.addVertex(vnew_test_editdistance, place);
	v1 = vnew_test_editdistance;
	
	place.mass_center = mcnew1_test_editdistance;
	gp_test_editdistance.addVertex(v1new_test_editdistance, place);
	v2 = v1new_test_editdistance;
	
	place.mass_center = mcnew2_test_editdistance;
	gp_test_editdistance.addVertex(v2new_test_editdistance, place);
	v3 = v2new_test_editdistance;
	
	place.mass_center = mcnew3_test_editdistance;
	gp_test_editdistance.addVertex(v3new_test_editdistance, place);
	v4 = v3new_test_editdistance;
	
	place.mass_center = mcnew4_test_editdistance;           
	gp_test_editdistance.addVertex(v4new_test_editdistance, place);
	v5 = v4new_test_editdistance;
	
	place.mass_center = mcnew5_test_editdistance;  
	gp_test_editdistance.addVertex(v5new_test_editdistance, place);
	v6 = v5new_test_editdistance;
	
	AASS::graphmatch::EdgePlace enew_test_editdistance;
	AASS::graphmatch::EdgePlace enew2_test_editdistance;
	AASS::graphmatch::EdgePlace enew3_test_editdistance;

	AASS::graphmatch::Gateway gt_test_editdistance;

	gp_test_editdistance.addEdge(vnew_test_editdistance, v1new_test_editdistance, enew_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v2new_test_editdistance, enew2_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v3new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v4new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v5new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(v3new_test_editdistance, v5new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
// 	gp_test_editdistance.addEdge(v3new_test_editdistance, v4new_test_editdistance, gt_test_editdistance, enew3_test_editdistance);
// 	gp_test_editdistance.addEdge(v3new_test_editdistance, v2new_test_editdistance, gt_test_editdistance, enew3_test_editdistance);
	
	
	AASS::topologicalmap::Vertex v;
	AASS::topologicalmap::Intersection_Graph inter;
	
	gp_test_editdistance.addCrossing(v1new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v1new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v2new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v3new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v3new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v5new_test_editdistance, v, inter);
}







BOOST_AUTO_TEST_CASE(trying)
{
	
	
	AASS::probabilisticmatching::VarianceEdgeNumber variance;
	
	AASS::graphmatch::GraphPlace gp_test_editdistance;
	createGraph(gp_test_editdistance);
	
	AASS::graphmatch::VertexPlace v1_test = v1;
	AASS::graphmatch::VertexPlace v2_test = v2;
// 	AASS::graphmatch::VertexPlace v3_test = v3;
	AASS::graphmatch::VertexPlace v4_test = v4;
	AASS::graphmatch::VertexPlace v5_test = v5;
	AASS::graphmatch::VertexPlace v6_test = v6;
	
	variance.calculate(gp_test_editdistance);
	
	double mean = (double) 12 / (double) 6;
	BOOST_CHECK_EQUAL(variance.getMean(), mean);
	
	BOOST_CHECK_EQUAL(variance.getFixedVariance(), 2);
	
	AASS::probabilisticmatching::PFFUUEdgeNumber en;
	
	double variance_model = gp_test_editdistance[v1].getVariance();
	double edge = gp_test_editdistance.getNumEdges(v1);
		
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model, edge+0.5, edge-0.5, 5), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model, edge+0.5, edge-0.5, 3), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model, edge+0.5, edge-0.5, 7), 1);
	
	double variance_model_second = gp_test_editdistance[v3].getVariance();
	double edge_second = gp_test_editdistance.getNumEdges(v3);
	
	std::cout << "Edge second " << edge_second << std::endl;
	
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model_second, edge_second+0.5, edge_second-0.5, 1), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model_second, edge_second+0.5, edge_second-0.5, 2), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model_second, edge_second+0.5, edge_second-0.5, 0), 1); //<- wrong
	
	
	AASS::graphmatch::GraphPlace gp_test_editdistance_2;
	createGraph(gp_test_editdistance_2);
	
	
	/************ TEST PF *******/
	
	AASS::probabilisticmatching::PFEdgeNumber pf;
	AASS::probabilisticmatching::VarianceEdgeNumber variance2;
	variance2.calculate(gp_test_editdistance);
	pf.setProbabilityTable(gp_test_editdistance, variance2);
	
	BOOST_CHECK_EQUAL(pf.size(), 6);
	
	BOOST_CHECK_EQUAL(pf.getProba(v1_test),1);
	BOOST_CHECK_EQUAL(pf.getProba(v2_test),1);
	
	
	
	/************************/
	
	AASS::probabilisticmatching::GraphProbaEdge en_2;
	
	en_2.clear();
	en_2.setProba(gp_test_editdistance, gp_test_editdistance_2);
	
// 	en_2.print();
	
// 	double probb = 0.28209479177387814 ;
	
	try{
		BOOST_CHECK_EQUAL(-1, en_2.getPUUFF(v1, v1));
	}
	catch(const std::exception& e){
		std::cout << std::endl;
		std::cout << "***************************** error but this means the test worked ****************************" << std::endl;
		std::cout << std::endl;
	}
	
// 	std::cout << "V1 with " << gp_test_editdistance_2.getNumEdges(v1) << "edges" << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v1_test, v1) >= en_2.getPUUFF(v1_test, v2));
// 	std::cout << "comparing to v2 ";
// 	std::cout << en_2.getPUUFF(v1_test, v1) << " " << en_2.getPUUFF(v1_test, v2) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v1_test, v1) >= en_2.getPUUFF(v1_test, v3));
// 	std::cout << "comparing to v3 ";
// 	std::cout << en_2.getPUUFF(v1_test, v1) << " " << en_2.getPUUFF(v1_test, v3) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v1_test, v1) >= en_2.getPUUFF(v1_test, v4));
// 	std::cout << "comparing to v4 ";
// 	std::cout << en_2.getPUUFF(v1_test, v1) << " " << en_2.getPUUFF(v1_test, v4) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v1_test, v1) >= en_2.getPUUFF(v1_test, v5));
// 	std::cout << "comparing to v5 ";
// 	std::cout << en_2.getPUUFF(v1_test, v1) << " " << en_2.getPUUFF(v1_test, v5) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v1_test, v1) >= en_2.getPUUFF(v1_test, v6));
// 	std::cout << "comparing to v6 ";
// 	std::cout << en_2.getPUUFF(v1_test, v1) << " " << en_2.getPUUFF(v1_test, v6) << std::endl;
// 	
// 	std::cout << "V2 with " << gp_test_editdistance_2.getNumEdges(v2) << "edges" <<  std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v2_test, v2) >= en_2.getPUUFF(v2_test, v1));
// 	std::cout << "comparing to v1 ";
// 	std::cout << en_2.getPUUFF(v2_test, v2) << " " << en_2.getPUUFF(v2_test, v1) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v2_test, v2) >= en_2.getPUUFF(v2_test, v3));
// 	std::cout << "comparing to v3 ";
// 	std::cout << en_2.getPUUFF(v2_test, v2) << " " << en_2.getPUUFF(v2_test, v3) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v2_test, v2) >= en_2.getPUUFF(v2_test, v4));
// 	std::cout << "comparing to v4 ";
// 	std::cout << en_2.getPUUFF(v2_test, v2) << " " << en_2.getPUUFF(v2_test, v4) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v2_test, v2) >= en_2.getPUUFF(v2_test, v5));
// 	std::cout << "comparing to v5 ";
// 	std::cout << en_2.getPUUFF(v2_test, v2) << " " << en_2.getPUUFF(v2_test, v5) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v2_test, v2) >= en_2.getPUUFF(v2_test, v6));
// 	std::cout << "comparing to v6 ";
// 	std::cout << en_2.getPUUFF(v2_test, v2) << " " << en_2.getPUUFF(v2_test, v6) << std::endl;
// 	
// 	std::cout << "V3 with " << gp_test_editdistance_2.getNumEdges(v3) << "edges" <<  std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v3_test, v3) >= en_2.getPUUFF(v3_test, v1));
// 	std::cout << "comparing to v1 ";
// 	std::cout << en_2.getPUUFF(v3_test, v3) << " " << en_2.getPUUFF(v3_test, v1) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v3_test, v3) >= en_2.getPUUFF(v3_test, v2));
// 	std::cout << "comparing to v2 ";
// 	std::cout << en_2.getPUUFF(v3_test, v3) << " " << en_2.getPUUFF(v3_test, v2) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v3_test, v3) >= en_2.getPUUFF(v3_test, v4));
// 	std::cout << "comparing to v4 ";
// 	std::cout << en_2.getPUUFF(v3_test, v3) << " " << en_2.getPUUFF(v3_test, v4) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v3_test, v3) >= en_2.getPUUFF(v3_test, v5));
// 	std::cout << "comparing to v5 ";
// 	std::cout << en_2.getPUUFF(v3_test, v3) << " " << en_2.getPUUFF(v3_test, v5) << std::endl;
// 	BOOST_CHECK(en_2.getPUUFF(v3_test, v3) >= en_2.getPUUFF(v3_test, v6));
// 	std::cout << "comparing to v6 ";
// 	std::cout << en_2.getPUUFF(v3_test, v3) << " " << en_2.getPUUFF(v3_test, v6) << std::endl;
	
	std::cout << "V4 with " << gp_test_editdistance_2.getNumEdges(v4) << "edges" <<  std::endl;
	BOOST_CHECK(en_2.getPUUFF(v4_test, v4) >= en_2.getPUUFF(v4_test, v1));
	en_2.getPUUFF(v4_test, v4);
	en_2.getPUUFF(v4_test, v1);
	std::cout << "comparing to v1 ";
	
	std::cout << en_2.getPUUFF(v4_test, v4) << " " << en_2.getPUUFF(v4_test, v1) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v4_test, v4) >= en_2.getPUUFF(v4_test, v2));
	
	std::cout << "comparing to v2 ";
	std::cout << en_2.getPUUFF(v4_test, v4) << " " << en_2.getPUUFF(v4_test, v2) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v4_test, v4) >= en_2.getPUUFF(v4_test, v3));
	std::cout << "comparing to v3 ";
	std::cout << en_2.getPUUFF(v4_test, v4) << " " << en_2.getPUUFF(v4_test, v3) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v4_test, v4) >= en_2.getPUUFF(v4_test, v5));
	std::cout << "comparing to v5 ";
	std::cout << en_2.getPUUFF(v4_test, v4) << " " << en_2.getPUUFF(v4_test, v5) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v4_test, v4) >= en_2.getPUUFF(v4_test, v6));
	std::cout << "comparing to v6 ";
	std::cout << en_2.getPUUFF(v4_test, v4) << " " << en_2.getPUUFF(v4_test, v6) << std::endl;

	std::cout << "V5 with " << gp_test_editdistance_2.getNumEdges(v5) << "edges" <<  std::endl;
	BOOST_CHECK(en_2.getPUUFF(v5_test, v5) >= en_2.getPUUFF(v5_test, v1));
	std::cout << "comparing to v1 ";
	std::cout << en_2.getPUUFF(v5_test, v5) << " " << en_2.getPUUFF(v5_test, v1) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v5_test, v5) >= en_2.getPUUFF(v5_test, v2));
	std::cout << "comparing to v2 ";
	std::cout << en_2.getPUUFF(v5_test, v5) << " " << en_2.getPUUFF(v5_test, v2) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v5_test, v5) >= en_2.getPUUFF(v5_test, v3));
	std::cout << "comparing to v3 ";
	std::cout << en_2.getPUUFF(v5_test, v5) << " " << en_2.getPUUFF(v5_test, v3) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v5_test, v5) >= en_2.getPUUFF(v5_test, v4));
	std::cout << "comparing to v4 ";
	std::cout << en_2.getPUUFF(v5_test, v5) << " " << en_2.getPUUFF(v5_test, v4) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v5_test, v5) >= en_2.getPUUFF(v5_test, v6));
	std::cout << "comparing to v6 ";
	std::cout << en_2.getPUUFF(v5_test, v5) << " " << en_2.getPUUFF(v5_test, v6) << std::endl;

	std::cout << "V6 with " << gp_test_editdistance_2.getNumEdges(v6) << "edges" << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v6_test, v6) >= en_2.getPUUFF(v6_test, v1));
	std::cout << "comparing to v1 ";
	en_2.getPUUFF(v6_test, v6);
	en_2.getPUUFF(v6_test, v1);
	std::cout << en_2.getPUUFF(v6_test, v6) << " " << en_2.getPUUFF(v6_test, v1) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v6_test, v6) >= en_2.getPUUFF(v6_test, v2));
	std::cout << "comparing to v2 ";
	std::cout << en_2.getPUUFF(v6_test, v6) << " " << en_2.getPUUFF(v6_test, v2) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v6_test, v6) >= en_2.getPUUFF(v6_test, v3));
	std::cout << "comparing to v3 ";
	std::cout << en_2.getPUUFF(v6_test, v6) << " " << en_2.getPUUFF(v6_test, v3) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v6_test, v6) >= en_2.getPUUFF(v6_test, v4));
	std::cout << "comparing to v4 ";
	std::cout << en_2.getPUUFF(v6_test, v6) << " " << en_2.getPUUFF(v6_test, v4) << std::endl;
	BOOST_CHECK(en_2.getPUUFF(v6_test, v6) >= en_2.getPUUFF(v6_test, v5));
	std::cout << "comparing to v5 ";
	std::cout << en_2.getPUUFF(v6_test, v6) << " " << en_2.getPUUFF(v6_test, v5) << std::endl;
	
	
	
	
	
	
	
	unit_test::VarianceTester lin;
	AASS::probabilisticmatching::VarianceEdgeNumber varvar;
	varvar.setEpsilon(5);
	lin.setNumberSimilar(varvar, gp_test_editdistance_2);
	
	std::vector<double> results_factor = lin.getFactors(varvar);
	
	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
	for (vp = boost::vertices(gp_test_editdistance_2.getGraph()); vp.first != vp.second; ++vp.first){
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "number fo edges " << gp_test_editdistance_2.getNumEdges(v) << std::endl;
	}
	std::cout << "Factors" << std::endl;
	for(size_t i = 0 ; i < results_factor.size() ; ++i){
		std::cout << results_factor[i] << " " << std::endl;
	}
	
	
	
	
}