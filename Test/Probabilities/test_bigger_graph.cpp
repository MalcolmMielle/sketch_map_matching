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
	
	
	AASS::probabilisticmatching::VarianceEdgeNumber variance;
	
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
	
	variance.calculate(gp_test_editdistance);
	
	std::cout << "Print variance " << std::endl;
	
	variance.print();
	
	double mean = (double) 12 / (double) 6;
	BOOST_CHECK_EQUAL(variance.getMean(), mean);
	
	BOOST_CHECK_EQUAL(variance.getFixedVariance(), 2);
	
	AASS::probabilisticmatching::PFFUUEdgeNumber en;
	
	double variance_model = gp_test_editdistance[v1].getVariance();
	double edge = gp_test_editdistance.getNumEdges(v1);
	
	int stop;
	
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model, edge + 0.5, edge - 0.5, 5), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model, edge + 0.5, edge - 0.5, 3), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model, edge + 0.5, edge - 0.5, 7), 1);
	
	double variance_model_second = gp_test_editdistance[v3].getVariance();
	double edge_second = gp_test_editdistance.getNumEdges(v3);
	
	std::cout << "Edge second " << edge_second << std::endl;
	
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model_second, edge_second + 0.5, edge_second - 0.5, 1), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model_second, edge_second + 0.5, edge_second - 0.5, 2), 1);
	BOOST_CHECK_EQUAL(en.normalDistribution(variance_model_second, edge_second + 0.5, edge_second - 0.5, 0), 1); //<- wrong
	
	
	AASS::graphmatch::GraphPlace gp_test_editdistance_2;
	createGraph(gp_test_editdistance_2);
	
	
	/************ TEST PF *******/
	
	AASS::probabilisticmatching::PFEdgeNumber pf;
	AASS::probabilisticmatching::VarianceEdgeNumber variance2;
	variance2.calculate(gp_test_editdistance);
	
	std::cout << "Print variance " << std::endl;
	variance.print();
	
	pf.setProbabilityTable(gp_test_editdistance, variance2);
	
	BOOST_CHECK_EQUAL(pf.size(), 6);
	
	BOOST_CHECK_EQUAL(pf.getProba(v1_test),1);
	BOOST_CHECK_EQUAL(pf.getProba(v2_test),1);
	
	
	
	/************************/
	
	AASS::probabilisticmatching::GraphProbaEdge en_2;
	
	en_2.clear();
	en_2.setProba(gp_test_editdistance, gp_test_editdistance_2);
	
// 	en_2.print();
	
	double probb = 0.28209479177387814 ;
	
	try{
		BOOST_CHECK_EQUAL(-1, en_2.getPUUFF(v1, v1));
	}
	catch(const std::exception& e){
		std::cout << std::endl;
		std::cout << "***************************** error but this means the test worked ****************************" << std::endl;
		std::cout << std::endl;
	}
	
	
	std::deque <AASS::graphmatch::VertexPlace > first_V;
	first_V.push_back(v1);
	first_V.push_back(v2);
	first_V.push_back(v3);
	first_V.push_back(v4);
	first_V.push_back(v5);
	first_V.push_back(v6);
	first_V.push_back(v7);
	first_V.push_back(v8);
	first_V.push_back(v9);
	first_V.push_back(v10);
	
		std::deque <AASS::graphmatch::VertexPlace > test_V;
	test_V.push_back(v1_test);
	test_V.push_back(v2_test);
	test_V.push_back(v3_test);
	test_V.push_back(v4_test);
	test_V.push_back(v5_test);
	test_V.push_back(v6_test);
	test_V.push_back(v7_test);
	test_V.push_back(v8_test);
	test_V.push_back(v9_test);
	test_V.push_back(v10_test);
	
	std::cout << "Print graph" << std::endl;
	for(size_t i = 0 ; i < first_V.size() ; i++){
		
		AASS::graphmatch::VertexPlace vvv = first_V[i];
		std::cout << "V" << i << " with " << gp_test_editdistance_2.getNumEdges(vvv) << "edges" << std::endl;
		
	}
	
	for(size_t i = 0 ; i < first_V.size() ; i++){
		
		AASS::graphmatch::VertexPlace vvv = first_V[i];
		AASS::graphmatch::VertexPlace vvv_test_same = test_V[i];
		
		std::cout << "V" << i << " with " << gp_test_editdistance_2.getNumEdges(vvv) << "edges" << std::endl;
		std::cout << "The same test V" << i << " with " << gp_test_editdistance_2.getNumEdges(vvv_test_same) << "edges" << std::endl;
		for(size_t j = 0 ; j < test_V.size() ; j++){
				
			AASS::graphmatch::VertexPlace vvv_test = test_V[j];
			std::cout << "comparing to v" << j << " with " << gp_test_editdistance_2.getNumEdges(vvv_test) << "edges" << std::endl;
			BOOST_CHECK(en_2.getPUUFF(vvv_test_same, vvv) >= en_2.getPUUFF(vvv_test, vvv));
			std::cout << en_2.getPUUFF(vvv_test_same, vvv) << " " << en_2.getPUUFF(vvv_test, vvv);
			std::cout << std::endl;
		}
		
		std::cout << std::endl;
		
	}
	
	
	
	
	
	//TEST REAL MAP
	
	
	cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");
	cv::Mat model = cv::imread("../Test/Sequences/fullmap.png");

	cv::threshold(mat_in, mat_in, 50, 255, CV_THRESH_BINARY_INV);
	cv::threshold(model, model, 50, 255, CV_THRESH_BINARY_INV);
	
	AASS::Thinker_Voronoi v_real;
	AASS::Thinker_Voronoi v_model;
	AASS::topologicalmap::MultipleLineFollower l_real;
	AASS::topologicalmap::MultipleLineFollower l_model;
	
	v_real.modeLaplaceVoro();
	v_real.setDownSample(1);
	v_model.modeLaplaceVoro();
	v_model.setDownSample(1);
	
	v_real.think(mat_in);
	v_model.think(model);
	
	cv::Mat v_real_res =  v_real.getResult();
	cv::Mat v_model_res =  v_model.getResult();
	
	l_real.inputMap(v_real.getResult());
	l_real.thin();
	l_model.inputMap(v_model.getResult());
	l_model.thin();
	
	AASS::topologicalmap::GraphList gl_real;
	gl_real = l_real.getGraph(0);
// 	gl_real.scale(2);
	AASS::topologicalmap::GraphList gl_model;
	gl_model = l_model.getGraph(0);
// 	gl_model.scale(2);
	
	AASS::graphmatch::PlaceExtractorList2Place place_real;
	AASS::graphmatch::PlaceExtractorList2Place place_model;
	place_real.inputMapIn(mat_in);
	place_real.inputGraph(gl_real);

	place_model.inputMapIn(model);
	place_model.inputGraph(gl_model);
	
	place_real.extract();
	place_model.extract();
	
	AASS::graphmatch::GraphPlace gp_real = place_real.getGraph();
	AASS::graphmatch::GraphPlace gp_model = place_model.getGraph();
	
	AASS::probabilisticmatching::VarianceEdgeNumber variance_second_input;
	AASS::probabilisticmatching::VarianceEdgeNumber variance_second_model;
	
	AASS::probabilisticmatching::GraphProbaEdge en_second;
	
	en_second.clear();
	en_second.setProba(gp_real, gp_model);
	en_second.setInput(gp_real);
	en_second.setModel(gp_model);
	
	AASS::probabilisticmatching::printProbaGraph(en_second, mat_in, mat_in);

	
}