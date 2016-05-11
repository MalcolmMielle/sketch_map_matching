#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
//TODO : test with empty graph


#include "VarianceEdgeNumber.hpp"
#include "GraphProbaEdge.hpp"
#include "Thinker_Voronoi.hpp"
#include "MultipleLineFollower.hpp"
#include "PlaceExtractorList2Place.hpp"
#include "ProbabilisticEditDistanceEdge.hpp"
#include "ProbabilisticMethod/Util.hpp"

AASS::graphmatch::VertexPlace v1;
AASS::graphmatch::VertexPlace v2;
AASS::graphmatch::VertexPlace v3;
AASS::graphmatch::VertexPlace v4;
AASS::graphmatch::VertexPlace v5;
AASS::graphmatch::VertexPlace v6;
AASS::graphmatch::VertexPlace v7;
AASS::graphmatch::VertexPlace v8;
AASS::graphmatch::VertexPlace v9;
AASS::graphmatch::VertexPlace v10;

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
	AASS::graphmatch::VertexPlace v6new_test_editdistance;
	AASS::graphmatch::VertexPlace v7new_test_editdistance;
	AASS::graphmatch::VertexPlace v8new_test_editdistance;
	AASS::graphmatch::VertexPlace v9new_test_editdistance;
	
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
	place.mass_center = mcnew1_test_editdistance;
	gp_test_editdistance.addVertex(v1new_test_editdistance, place);
	place.mass_center = mcnew2_test_editdistance;
	gp_test_editdistance.addVertex(v2new_test_editdistance, place);
	place.mass_center = mcnew3_test_editdistance;
	gp_test_editdistance.addVertex(v3new_test_editdistance, place);
	place.mass_center = mcnew4_test_editdistance;           
	gp_test_editdistance.addVertex(v4new_test_editdistance, place);
	place.mass_center = mcnew5_test_editdistance;           
	gp_test_editdistance.addVertex(v5new_test_editdistance, place);
	gp_test_editdistance.addVertex(v6new_test_editdistance, place);
	gp_test_editdistance.addVertex(v7new_test_editdistance, place);
	gp_test_editdistance.addVertex(v8new_test_editdistance, place);
	gp_test_editdistance.addVertex(v9new_test_editdistance, place);
	
	v1 = vnew_test_editdistance;
	v2 = v1new_test_editdistance;
	v3 = v2new_test_editdistance;
	v4 = v3new_test_editdistance;
	v5 = v4new_test_editdistance;
	v6 = v5new_test_editdistance;
	v7 = v6new_test_editdistance;
	v8 = v7new_test_editdistance;
	v9 = v8new_test_editdistance;
	v10 = v9new_test_editdistance;
	
	
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
	gp_test_editdistance.addEdge(v3new_test_editdistance, v4new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(v3new_test_editdistance, v2new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	
	gp_test_editdistance.addEdge(v6new_test_editdistance, v2new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(v6new_test_editdistance, v4new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	
	gp_test_editdistance.addEdge(v7new_test_editdistance, v2new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(v7new_test_editdistance, v3new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(v7new_test_editdistance, vnew_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	
	gp_test_editdistance.addEdge(v8new_test_editdistance, v2new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	
	
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
	
	
	
	AASS::graphmatch::GraphPlace gp_test_editdistance;
	createGraph(gp_test_editdistance);
	
	AASS::graphmatch::VertexPlace v1_test = v1;
	AASS::graphmatch::VertexPlace v2_test = v2;
	AASS::graphmatch::VertexPlace v3_test = v3;
	AASS::graphmatch::VertexPlace v4_test = v4;
	AASS::graphmatch::VertexPlace v5_test = v5;
	AASS::graphmatch::VertexPlace v6_test = v6;
	AASS::graphmatch::VertexPlace v7_test = v7;
	AASS::graphmatch::VertexPlace v8_test = v8;
	AASS::graphmatch::VertexPlace v9_test = v9;
	AASS::graphmatch::VertexPlace v10_test = v10;
	
	AASS::graphmatch::GraphPlace gp_test_editdistance_v2;
	createGraph(gp_test_editdistance_v2);
	
	/*
	 * links 
	 * 1 -> 2, 3, 4, 5, 6, 8
	 * 2 -> 1, 6, 5, 3
	 * 3 -> 1, 2, 4, 7, 8, 9
	 * 4 -> 1, 3, 8
	 * 5 -> 1, 2, 7
	 * 6 -> 1, 2
	 * 7 -> 1, 3, 5
	 * 8 -> 1, 3, 4
	 * 9 -> 3
	 * 10 ->
	 */
	
	AASS::graphmatch::Hypothese hyp;
	AASS::graphmatch::Match m(v1_test, v1);
	m.setCost(2);
	
	AASS::graphmatch::Match m2(v2_test, v2);
	m2.setCost(3);
	
	AASS::graphmatch::Match m3(v3_test, v3);
	m3.setCost(1.5);
	
	AASS::graphmatch::Match m4(v4_test, v10);
	m4.setCost(1);
	
	AASS::graphmatch::Match m5(v7_test, v5);
	m5.setCost(0.25);
	
	hyp.push_back(m);
	hyp.push_back(m2);
	hyp.push_back(m3);
	hyp.push_back(m4);
	
	AASS::graphmatch::Hypothese zone;
	
	int count = hyp.getSizeSimilarZone(gp_test_editdistance, m.getFirst(), zone);
	
	BOOST_CHECK_EQUAL(count , 3 );
	BOOST_CHECK_EQUAL(zone.size(), 3);
	BOOST_CHECK_EQUAL(zone[0], m);
	BOOST_CHECK_EQUAL(zone[1], m3);
	BOOST_CHECK_EQUAL(zone[2], m4);
	
	zone.clear();
	count = hyp.getSizeSimilarZone(gp_test_editdistance_v2, m.getSecond(), zone);
	BOOST_CHECK_EQUAL(count , 2 );
	BOOST_CHECK_EQUAL(zone.size(), 2);
	BOOST_CHECK_EQUAL(zone[0], m);
	BOOST_CHECK_EQUAL(zone[1], m3);
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m2.getFirst()), 4 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m2.getSecond()), 3 );
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m3.getFirst()), 2 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m3.getSecond()), 1 );
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m4.getFirst()), 1 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m4.getSecond()), 1 );
	
	hyp.push_back(m5);
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m.getFirst()), 4 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m.getSecond()), 3 );
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m2.getFirst()), 5 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m2.getSecond()), 4 );
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m3.getFirst()), 3 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m3.getSecond()), 1 );
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m4.getFirst()), 1 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m4.getSecond()), 1 );
	
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance, m5.getFirst()), 1 );
	BOOST_CHECK_EQUAL(hyp.getSizeSimilarZone(gp_test_editdistance_v2, m5.getSecond()), 1 );
	

	
}