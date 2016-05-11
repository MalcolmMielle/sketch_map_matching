#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "Hypothese.hpp"
#include "GraphMatcherOld.hpp"
#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "JunctionAndDeadEnds.hpp"
#include "PlaceExtractorList2Place.hpp"

AASS::graphmatch::VertexPlace v1;
AASS::graphmatch::VertexPlace v2;
AASS::graphmatch::VertexPlace v3;

AASS::graphmatch::VertexPlace v4;

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
	cv::Point2i mcnew4_test_editdistance(50, 150); //r    3
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
	
	v1 = vnew_test_editdistance;
	v2 = v1new_test_editdistance;
	v3 = v2new_test_editdistance;
	
	v4 = v3new_test_editdistance;
	
	
	AASS::graphmatch::EdgePlace enew_test_editdistance;
	AASS::graphmatch::EdgePlace enew2_test_editdistance;
	AASS::graphmatch::EdgePlace enew3_test_editdistance;

	AASS::graphmatch::Gateway gt_test_editdistance;

	gp_test_editdistance.addEdge(enew_test_editdistance, vnew_test_editdistance, v1new_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(enew2_test_editdistance,vnew_test_editdistance, v2new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v3new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v4new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v5new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,v3new_test_editdistance, v5new_test_editdistance,  gt_test_editdistance);
	
	AASS::vodigrex::SimpleNode inter;
	bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex vert;
	std::pair <AASS::vodigrex::SimpleNode, bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex > pair(inter, vert);

	
	gp_test_editdistance[v1new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v1new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v2new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v3new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v3new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v5new_test_editdistance].landmarks.push_back( pair);

}


void createGraphtwo(AASS::graphmatch::GraphPlace& gp_test_editdistance){

	
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
	cv::Point2i mcnew4_test_editdistance(50, 150); //r    3
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
	
	g2_v1 = vnew_test_editdistance;
	g2_v2 = v1new_test_editdistance;
	g2_v3 = v2new_test_editdistance;
	
	g2_v4 = v3new_test_editdistance;
	
	AASS::graphmatch::EdgePlace enew_test_editdistance;
	AASS::graphmatch::EdgePlace enew2_test_editdistance;
	AASS::graphmatch::EdgePlace enew3_test_editdistance;

	AASS::graphmatch::Gateway gt_test_editdistance;

	gp_test_editdistance.addEdge(enew_test_editdistance, vnew_test_editdistance, v1new_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(enew2_test_editdistance, vnew_test_editdistance, v2new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance, vnew_test_editdistance, v3new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance, vnew_test_editdistance, v4new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance, vnew_test_editdistance, v5new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance, v3new_test_editdistance, v5new_test_editdistance, gt_test_editdistance);

	AASS::vodigrex::SimpleNode inter;
	bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex vert;
	std::pair <AASS::vodigrex::SimpleNode, bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex > pair(inter, vert);

	
	gp_test_editdistance[v1new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v1new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v1new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v2new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v3new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v5new_test_editdistance].landmarks.push_back( pair);
	
}



BOOST_AUTO_TEST_CASE(trying)
{
	
	AASS::graphmatch::GraphPlace gp;
	AASS::graphmatch::GraphPlace gp_model;
	
	createGraph(gp);
	createGraphtwo(gp_model);
	
	AASS::graphmatch::AllKeypointJunctionDeadEnd allkey;
	addTypeVertex(gp, allkey);
	addTypeVertex(gp_model, allkey);
	
	std::cout << "Pushing hyo" << std::endl;
	AASS::graphmatch::Hypothese hypo;
	hypo.push_back(AASS::graphmatch::Match(v2, g2_v2));
	hypo.push_back(AASS::graphmatch::Match(v1, g2_v1));
	hypo.push_back(AASS::graphmatch::Match(v4, g2_v4));
	
	std::cout << "up dist" << std::endl;
	int distance = hypo.updateDistance(gp, gp_model);
	
	BOOST_CHECK_EQUAL(distance, 14);
	
	std::cout << "DISTANCE : " << distance << std::endl;
		
// 	co.drawHypo(gp, gp_model, hypo.getMatches(), "hypo");
	
	
	hypo.push_back(AASS::graphmatch::Match(v3, g2_v3));
	
	int distance_2 = hypo.updateDistance(gp, gp_model);
	
	std::cout << "DISTANCE : " << distance_2 << std::endl;
	
	BOOST_CHECK_EQUAL(distance_2, 10);
	cv::waitKey(0);
	
	
	
}