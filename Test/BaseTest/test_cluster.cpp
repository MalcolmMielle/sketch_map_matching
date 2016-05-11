#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "Cluster.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "Thinker_Voronoi.hpp"

AASS::graphmatch::VertexPlace v1;
AASS::graphmatch::VertexPlace v2;
AASS::graphmatch::VertexPlace v3;

AASS::graphmatch::VertexPlace v4;

AASS::graphmatch::VertexPlace g2_v1;
AASS::graphmatch::VertexPlace g2_v2;
AASS::graphmatch::VertexPlace g2_v3;

AASS::graphmatch::VertexPlace g2_v4;

AASS::graphmatch::EdgePlace e1;


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
	cv::Point2i mcnew4_test_editdistance(80, 150); //r    3
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
	
	gp_test_editdistance.addEdge(vnew_test_editdistance, v1new_test_editdistance, enew_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v2new_test_editdistance, enew2_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v3new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v4new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(vnew_test_editdistance, v5new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(v3new_test_editdistance, v5new_test_editdistance, enew3_test_editdistance, gt_test_editdistance);
	
	e1 = enew_test_editdistance;
	
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
	
	g2_v1 = vnew_test_editdistance;
	g2_v2 = v1new_test_editdistance;
	g2_v3 = v2new_test_editdistance;
	
	g2_v4 = v3new_test_editdistance;
	
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

	
	AASS::topologicalmap::Vertex v;
	AASS::topologicalmap::Intersection_Graph inter;
	
	gp_test_editdistance.addCrossing(v1new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v1new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v1new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v2new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v3new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	gp_test_editdistance.addCrossing(v4new_test_editdistance, v, inter);
	
	gp_test_editdistance.addCrossing(v5new_test_editdistance, v, inter);
}


BOOST_AUTO_TEST_CASE(trying)
{
	

	
	cv::Mat mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
	cv::Mat draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	cv::Mat draw_graph_reduced_2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);

	
	/***********************/
	
	
	AASS::graphmatch::GraphPlace fp2;
	AASS::graphmatch::GraphPlace fpmodel2;
	
	createGraph(fp2);
	createGraphtwo(fpmodel2);
	
	draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	draw_graph_reduced_2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	fp2.drawSpecial(draw_graph_reduced);
	fpmodel2.drawSpecial(draw_graph_reduced_2);
	cv::imshow("fp2 begin", draw_graph_reduced);
	cv::imshow("fp2 bgein model", draw_graph_reduced_2);
	cv::waitKey(0);
	
	AASS::graphmatch::Hypothese h22;
// 	h22.push_back(AASS::graphmatch::Match(v1, g2_v1));
	h22.push_back(AASS::graphmatch::Match(v2, g2_v2));
	h22.push_back(AASS::graphmatch::Match(v3, g2_v3));
	
	std::deque < AASS::graphmatch::Hypothese > dhyp2;
	AASS::graphmatch::Hypothese h222;
	h222.push_back(AASS::graphmatch::Match(v4, g2_v4));
	dhyp2.push_back(h222);
	
	AASS::graphmatch::Cluster cl(dhyp2);
	
	BOOST_CHECK_EQUAL(true, cl.isCompatible(h22, fp2, fpmodel2));
	
	
	draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	draw_graph_reduced_2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	fp2.drawSpecial(draw_graph_reduced);
	fpmodel2.drawSpecial(draw_graph_reduced_2);
	cv::imshow("fp2 end", draw_graph_reduced);
	cv::imshow("fp2 model end", draw_graph_reduced_2);
	cv::waitKey(0);
	
	
	AASS::graphmatch::Cluster cl_reduce(dhyp2);
	
	cl_reduce.reduceTEST(h22, fp2, fpmodel2);
	
	

	
	
}