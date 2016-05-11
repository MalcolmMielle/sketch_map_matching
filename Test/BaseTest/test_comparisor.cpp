#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "Cluster.hpp"
#include "GraphMatcherClusterFiltered.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
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
	gp_test_editdistance.addEdge(enew3_test_editdistance, v3new_test_editdistance, v5new_test_editdistance, gt_test_editdistance);
	
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

	gp_test_editdistance.addEdge(enew_test_editdistance, vnew_test_editdistance, v1new_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(enew2_test_editdistance,vnew_test_editdistance, v2new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v3new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v4new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v5new_test_editdistance,  gt_test_editdistance);
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
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	gp_test_editdistance[v4new_test_editdistance].landmarks.push_back( pair);
	
	gp_test_editdistance[v5new_test_editdistance].landmarks.push_back( pair);
	
}


BOOST_AUTO_TEST_CASE(trying)
{
	
	cv::Mat mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
// 	cv::Mat model = cv::imread("../Test/TEST_COMPARISON/TEST1/model/model.png");
//  	
// 	cv::Mat tmp = cv::Mat::zeros(mat_in.size(), CV_8UC1);
// 	cv::Mat tmp2 = cv::Mat::zeros(mat_in.size(), CV_8UC1);
// 	cv::Mat tmpp = cv::Mat::zeros(mat_in.size(), CV_8UC1);
// 	cv::Mat tmpp2 = cv::Mat::zeros(mat_in.size(), CV_8UC1);
// 	
// 	AASS::topologicalmap::GraphList glist;
// 	AASS::topologicalmap::GraphList glist2;
// 	
// 	Thinker_Voronoi v_real;
// 	Thinker_Voronoi v_model;
// 	
// 	Map map(v_real);
// 	Map map_model(v_model);
// 	
// 	AASS::topologicalmap::LineFollower l_real;
// 	AASS::topologicalmap::LineFollower l_model;
// 	
// 	
// 	//Voronoi lines
// 	v_real.modeLaplaceVoro();
// 	v_real.setDownSample(1);
// 	v_model.modeLaplaceVoro();
// 	v_model.setDownSample(1);
// 	
// 	v_real.think(mat_in);
// 	v_model.think(model);
// 	
// 	
// 	//Line follower
// 	l_real.inputMap(v_real.getResult());
// 	l_real.thin();
// 	l_model.inputMap(v_model.getResult());
// 	l_model.thin();
// 	
// 	//Getting graphs
// 	AASS::topologicalmap::GraphList gl_real;
// 	gl_real = l_real.getGraph();
// 	gl_real.scale(2);
// 	AASS::topologicalmap::GraphList gl_model;
// 	gl_model = l_model.getGraph();
// 	gl_model.scale(2);
// 	
// 	
// 	//Extract places
// 	AASS::topologicalmap::PlaceExtractor place_real;
// 	AASS::topologicalmap::PlaceExtractor place_model;
// 	place_real.inputMapIn(mat_in);
// 	place_real.inputPreviousInfo(gl_real);
// 	place_real.init();
// 	place_model.inputMapIn(model);
// 	place_model.inputPreviousInfo(gl_model);
// 	place_model.init();
// 	
// 	AASS::topologicalmap::GraphList graph_pruned = gl_real;
// 	AASS::topologicalmap::GraphList graph_pruned_model = gl_model;
// 	place_real.prunePreviousGraph(graph_pruned);
// 	place_model.prunePreviousGraph(graph_pruned_model);
// 	
// 	place_real.extract();
// 	place_model.extract();
// 	AASS::graphmatch::GraphPlace gp_real = place_real.getGraph();
// 	AASS::graphmatch::GraphPlace gp_model = place_model.getGraph();
// 	
// 	
// 	
// 	//Create graphmatch
// 	map.setObstacleMat(mat_in);
// 	map_model.setObstacleMat(model);
// 	
// 	graphmatch::GraphMatcherCluster graphmatch(map, map_model);
// 	
// 	
// 	std::deque< 
// 			graphmatch::Match > places_pair;
// 	
// 	graphmatch.pairWiseMatch(gp_real, gp_real, places_pair);
// 	
// 	std::deque< 
// 			graphmatch::Match >::iterator it;
// 				
// 	for(it = places_pair.begin() ; it != places_pair.end(); it++){
// 		std::cout << "MATCH of room : " << gp_real.isRoom( (*it).getFirst() ) << " with number of edge " << gp_real.getNumEdges( (*it).getFirst()) << std::endl;
// 		gp_real.print( (*it).getFirst());
// 		gp_real.print( (*it).getSecond());
// 		BOOST_CHECK_EQUAL(gp_real.isRoom( (*it).getFirst()), gp_real.isRoom( (*it).getSecond()));
// 		BOOST_CHECK_EQUAL(gp_real.getNumEdges( (*it).getFirst()), gp_real.getNumEdges( (*it).getSecond()));
// 		std::cout << std::endl;
// 	}
// 	
// 	
// 	/*************SECOND TEST***************/
// 	
// 	AASS::graphmatch::GraphPlace g1;
// 	AASS::graphmatch::GraphPlace g2;
// 	
// 	createGraph(g1);	
// 	createGraphtwo(g2);
// 	
// 	cv::Mat draw_graph = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	g1.drawSpecial(draw_graph);
// 	cv::Mat draw_graph2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	g2.drawSpecial(draw_graph2);
// 	cv::imshow("first", draw_graph);
// 	cv::imshow("second", draw_graph2);
// 	
// 	
// 	graphmatch::Cluster clu;
// 	graphmatch::Hypothese hypo;
// 	hypo.push_back(graphmatch::Match(v2, v2));
// 	hypo.push_back(graphmatch::Match(v4, v4));
// 	
// 	std::deque < AASS::graphmatch::VertexPlace > dd;
// 	std::deque < AASS::graphmatch::VertexPlace > dd2;
// 	
// 	dd.push_back(v2);
// 	dd.push_back(v1);
// 	dd.push_back(v3);
// 	
// 	dd2.push_back(v4);
// 	
// 	g1.reduce(dd);
// 	
// 	g1.print();
// 	
// 	g1.labelAll(dd);
// 	g1.labelAll(dd2);
// 	
// 	std::cout << "Removing labels" << std::endl;
// 	g1.removeAllFalseLabel();
// 	
// 	g1.print();
// 	
// 	BOOST_CHECK_EQUAL(g1.getNumEdges(), 1);
// 	
// 	std::cout << "IsSame" << std::endl;
// 	clu.setGraph(g1);
// 	clu.setGraphModel(g1);
// 	BOOST_CHECK_EQUAL(clu.isSame(hypo), true);
// 	
// 	
// 	std::deque < AASS::graphmatch::VertexPlace > g2dd;
// 	std::deque < AASS::graphmatch::VertexPlace > g2dd2;
// 	
// 	g2dd.push_back(g2_v2);
// 	g2dd.push_back(g2_v1);
// 	g2dd.push_back(g2_v3);
// 	
// 	g2dd2.push_back(g2_v4);
// 	
// 	g2.reduce(g2dd);
// 	
// 	g2.print();
// 	
// 	g2.labelAll(g2dd);
// 	g2.labelAll(g2dd2);
// 	
// 	std::cout << "Removing labels" << std::endl;
// 	g2.removeAllFalseLabel();
// 	
// 	graphmatch::Hypothese hypo2;
// 	hypo2.push_back(graphmatch::Match(v2, g2_v2));
// 	hypo2.push_back(graphmatch::Match(v4, g2_v4));
// 	
// 	clu.setGraph(g1);
// 	clu.setGraphModel(g2);
// 	BOOST_CHECK_EQUAL(clu.isSame(hypo2), true);
// 	
// 	cv::Mat draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	cv::Mat draw_graph_reduced_2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	g1.drawSpecial(draw_graph_reduced);
// 	g2.drawSpecial(draw_graph_reduced_2);
// 	cv::imshow("reduced", draw_graph_reduced);
// 	cv::imshow("reduced_2", draw_graph_reduced_2);
// 	cv::waitKey(0);
// 	
// 	g2.print();
// 	std::cout << std::endl << "SECOND GRAPH " << std::endl;
// 	g1.print();
// 	
// 	/* Removing all the vertex outside of the hypo*/
	
	
	/*************THIRD CLUSTER REDUCE TEST***************/
	
// 	AASS::graphmatch::GraphPlace g1_clu;
// 	AASS::graphmatch::GraphPlace g2_clu;
// 	
// 	createGraph(g1_clu);	
// 	createGraphtwo(g2_clu);
// 	
// 	graphmatch::Hypothese hypo_clu_2_graph2;
// 	hypo_clu_2_graph2.push_back(graphmatch::Match(v2, g2_v2));
// 	hypo_clu_2_graph2.push_back(graphmatch::Match(v1, g2_v1));
// 	hypo_clu_2_graph2.push_back(graphmatch::Match(v3, g2_v3));
// 	
// 	
// 	graphmatch::Hypothese hypo2_clu_2_graph2;
// 	hypo2_clu_2_graph2.push_back(graphmatch::Match(v4, g2_v4));
// 	
// 	graphmatch::Cluster clu_clu2;
// 	clu_clu2.setGraph(g1_clu);
// 	clu_clu2.setGraphModel(g2_clu);
// 	
// 	clu_clu2.push_back(hypo_clu_2_graph2);
// 	
// // 	clu_clu2.reduce(hypo2_clu_2_graph2, g1_clu, g2_clu);
// 	
// 	BOOST_CHECK_EQUAL(clu_clu2.isSame(hypo2_clu_2_graph2, g1_clu, g2_clu), false);
// 	BOOST_CHECK_EQUAL(clu_clu2.isCompatible(hypo2_clu_2_graph2, g1_clu, g2_clu), true);
// 	
// 	cv::Mat draw_graph_reduced_clu2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	cv::Mat draw_graph_reduced_2_clu2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	g1_clu.drawSpecial(draw_graph_reduced_clu2);
// 	g2_clu.drawSpecial(draw_graph_reduced_2_clu2);
// 	cv::imshow("reduced", draw_graph_reduced_clu2);
// 	cv::imshow("reduced_2", draw_graph_reduced_2_clu2);
// 	cv::waitKey(0);
// 	
// 	g2_clu.print();
// 	std::cout << std::endl << "SECOND GRAPH " << std::endl;
// 	g1_clu.print();
	
	
	/********************************************************************************/
	
	
	cv::Mat model;
	mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
	model = cv::imread("../Test/TEST_COMPARISON/TEST1/model/model.png");
	
	AASS::vodigrex::ThinkerVoronoi v_real;
	AASS::vodigrex::ThinkerVoronoi v_model;
	AASS::vodigrex::LineFollowerGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l_real;
	AASS::vodigrex::LineFollowerGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l_model;
	
	v_real.modeLaplaceVoro();
	v_real.setDownSample(1);
	v_model.modeLaplaceVoro();
	v_model.setDownSample(1);
	
	v_real.think(mat_in);
	v_model.think(model);
	
	l_real.inputMap(v_real.getResult());
	l_real.thin();
	l_model.inputMap(v_model.getResult());
	l_model.thin();
	
	AASS::topologicalmap::GraphLine gl_real(l_real.getGraph());
	gl_real.scale(2);
	AASS::topologicalmap::GraphLine gl_model(l_model.getGraph());
	gl_model.scale(2);
	
	AASS::graphmatch::PlaceExtractorList2Place place_real;
	AASS::graphmatch::PlaceExtractorList2Place place_model;
	place_real.inputMapIn(mat_in);
	place_real.inputGraph(gl_real);
	place_model.inputMapIn(model);
	place_model.inputGraph(gl_model);
	
// 	AASS::topologicalmap::GraphList graph_pruned = gl_real;
// 	AASS::topologicalmap::GraphList graph_pruned_model = gl_model;
// 	place_real.prunePreviousGraph(graph_pruned);
// 	place_model.prunePreviousGraph(graph_pruned_model);
	
	place_real.extract();
	place_model.extract();
	
	AASS::graphmatch::GraphPlace gp_real = place_real.getGraph();	
	AASS::graphmatch::GraphPlace gp_model = place_model.getGraph();
	
	cv::Mat draw_pruned = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	cv::Mat draw_pruned_model = cv::Mat::zeros(model.size(), CV_8UC3);
	
	gl_real.draw(draw_pruned);
	gl_model.draw(draw_pruned_model);
	
	cv::imshow("pruned in", draw_pruned);
	cv::imshow("pruned model", draw_pruned_model);
	
	cv::waitKey(0);

	
	AASS::graphmatch::GraphMatcherNeighbor graphmatch;
			
	std::cout << "Start of planar edit distance" << std::endl;

	graphmatch.planarEditDistanceAlgorithm(gp_real, gp_model);
	
	std::deque<	
			AASS::graphmatch::Hypothese
		> hypothesis_final = graphmatch.getResult();
		
	graphmatch.sort(hypothesis_final);
		
		
	std::deque< 
		AASS::graphmatch::Match
		> list_result;
		
	if(hypothesis_final.size() > 0){
		
		for(size_t i = 0 ; i < hypothesis_final.size() ; i++){
			
			int best = hypothesis_final[i].getDist();
			list_result = hypothesis_final[i].getMatches();
			
			//PRINTING
			cv::Mat draw_links = cv::Mat::zeros(model.size(), CV_8UC3);
			cv::Mat draw_graph = cv::Mat::zeros(mat_in.size(), CV_8UC3);
			cv::Mat draw_graph_model = cv::Mat::zeros(model.size(), CV_8UC3);
			
			cv::Mat draw_links_other = cv::Mat::zeros(model.size(), CV_8UC3);
			cv::Mat draw_graph_other = cv::Mat::zeros(mat_in.size(), CV_8UC3);
			cv::Mat draw_graph_model_other = cv::Mat::zeros(model.size(), CV_8UC3);
			
			cv::Scalar color;
			cv::RNG rrng(12345);
				
			if(draw_links.channels() == 1){
				color = rrng.uniform(50, 255);
			}
			
			else if(draw_links.channels() == 3){
				color[1] = rrng.uniform(50, 255);
				color[2] = rrng.uniform(50, 255);
				color[3] = rrng.uniform(50, 255);
			}
			
			cv::Scalar color_model;
				
			if(draw_links.channels() == 1){
				color_model = 100;
			}
			
			else if(draw_links.channels() == 3){
				color_model[1] = 100;
				color_model[2] = 100;
				color_model[3] = 100;
			}
			
			cv::Scalar color_one;
				
			if(draw_links.channels() == 1){
				color_one = 255;
			}
			
			else if(draw_links.channels() == 3){
				color_one[1] = 255;
				color_one[2] = 255;
				color_one[3] = 255;
			}
			
			
			for(size_t ii = 0 ; ii < list_result.size() ; ii++ ){
				std::cout << "this : ";
				gp_real.print(list_result[ii].getFirst());
				std::cout << " linked to this : " ;
				gp_model.print(list_result[ii].getSecond());
				std::cout << std::endl;
				
				cv::line(draw_links, gp_real[list_result[ii].getFirst()].mass_center, gp_model[list_result[ii].getSecond()].mass_center, color);
				cv::circle(draw_links, gp_model[list_result[ii].getSecond()].mass_center, 10, color_model, 3);
				cv::circle(draw_links, gp_real[list_result[ii].getFirst()].mass_center, 10, color_one, 3);
				
			}
			
			gp_real.drawSpecial(draw_graph);
			gp_model.drawSpecial(draw_graph_model);
			
			cv::imshow("links at the end", draw_links);
			cv::imshow("graph place", draw_graph);
			cv::imshow("model graph place", draw_graph_model);
			
			
			
			hypothesis_final[i].drawHypo(gp_real, gp_model, mat_in, model, "ALL FINAL");
			std::cout << "Distance is : " << best << std::endl;
			cv::waitKey(0);
		// 		graphmatch.drawHypo(gp_real, gp_model, list_result_other, "ALL FINAL other paper");
			
			
			
		}
	}
	else{
		std::cout << "No result, this is weird" << std::endl;
		
	}
// 	
	
	
// 	/* TEST CLUSTERING */
// 	Thinker_Voronoi tv;
// 	Map m(tv);
// 	
// 	AASS::graphmatch::GraphPlace g11;
// 	AASS::graphmatch::GraphPlace g22;
// 	
// 	createGraph(g11);	
// 	createGraphtwo(g22);
// 
// 
// 	graphmatch::GraphMatcherCluster compv2(m, m);
// 	std::deque < graphmatch::Hypothese > dhyp;
// 	
// 	graphmatch::Hypothese hypo4;
// 	hypo4.push_back(graphmatch::Match(v2, v2));
// 	hypo4.push_back(graphmatch::Match(v1, v1));
// 	hypo4.push_back(graphmatch::Match(v3, v3));
// 	
// 	graphmatch::Hypothese hypo3;
// 	hypo3.push_back(graphmatch::Match(v4, v4));
// 	
// 	dhyp.push_back(hypo4);
// 	dhyp.push_back(hypo3);
// 	
// 	compv2.init(g11, g11);
// 	
// 	compv2.cluster(dhyp);
	
// 	hypo.push_back(graphmatch::Match() );
	
	
// 	int corner = 0;
// 	int rooms = 0 ;
// 	
// 	int landmarks = 0;
// 	int junctions = 0;
	
// 	graphmatch.countingPlace(corner, rooms, m.getGraphPlace() );
// 	graphmatch.countingVoronoiLines(landmarks, junctions, m.getGraphLines() );
// 	
// 	BOOST_CHECK_EQUAL(corner, 2);
// 	BOOST_CHECK_EQUAL(rooms, 4);
// 	BOOST_CHECK_EQUAL(graphmatch.structuralDifferencies(), 2);
	
	
// 	for(int i = 0 ; i < 3 ; i++){
// 		std::cout << "TEST" << std::endl << std::endl;
// 		graphmatch.changeModeGraphFusion();
// 		graphmatch.thinkALL();
// 		std::cout << "Mode fusion " << graphmatch.getMapModel().getModeGraphFusionName()<<std::endl;
// 		std::cout << "DIFFERENCES " << graphmatch.structuralDifferencies() << std::endl;
// 	}
	
// 	graphmatch.planarEditDistanceAlgorithm(gp_real, gp_model);
	
	
	
	
	
// 	
// 	
// 	
// 	std::cout << "EXPANSION TEST" << std::endl;
// 	std::deque<std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > > > H;
// 	
// 	graphmatch.growHypotheses(gp, gp, H);
// 	
// 	std::cout << "END RESULT" << std::endl;
// 	
// 	std::deque< std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > > >::iterator ittt;
// 	std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > >::iterator itt;
// 	
// 	cv::waitKey(0);
// 	
// 	for(ittt = H.begin() ; ittt != H.end() ; ittt++){
// 		
// 		std::cout << "Hypothesis" << std::endl << std::endl;
// 		cv::RNG rng(12345);
// 		cv::Mat temp = cv::Mat::zeros(mapgate.size(), CV_8UC1);
// 		cv::Mat hypoone = cv::Mat::zeros(mapgate.size(), CV_8UC1);
// 		cv::Mat hypotwo = cv::Mat::zeros(mapgate.size(), CV_8UC1);
// 		
// 		for(itt = (*ittt).begin() ; itt != (*ittt).end() ; itt++){
// 			cv::Scalar color;
// 			
// 			if(temp.channels() == 1){
// 				color = rng.uniform(50, 255);
// 			}
// 			
// 			else if(tmp.channels() == 3){
// 				color[1] = rng.uniform(50, 255);
// 				color[2] = rng.uniform(50, 255);
// 				color[3] = rng.uniform(50, 255);
// 			}
// 			
// 			std::cout <<"Match" << std::endl;
// 			gp.print( (*itt).first );
// 			gp.print( (*itt).second );
// 			
// 			gp.draw(temp, (*itt).first, color);
// 			gp.draw(temp, (*itt).second, color);
// 			
// 			cv::line(temp, gp[(*itt).first].mass_center, gp[(*itt).second].mass_center, color);
// 			
// 		}
// 		itt = (*ittt).begin();
// 		itt++;
// 		cv::Scalar color;
// 			
// 		if(temp.channels() == 1){
// 			color = rng.uniform(50, 255);
// 		}
// 		
// 		else if(tmp.channels() == 3){
// 			color[1] = rng.uniform(50, 255);
// 			color[2] = rng.uniform(50, 255);
// 			color[3] = rng.uniform(50, 255);
// 		}
// 		
// 		for( ; itt != (*ittt).end() ; itt++){
// 			cv::line(hypoone, gp[(*(itt-1)).first].mass_center, gp[(*itt).first].mass_center, color);
// 			cv::line(hypotwo, gp[(*(itt-1)).second].mass_center, gp[(*itt).second].mass_center, color);
// 		}
// 		
// 		cv::imshow("hypo one", hypoone);
// 		cv::imshow("hypo two", hypotwo);
// 		cv::imshow("the hypo correspondence", temp);
// 		cv::waitKey(0);
// 		
// 	}
// 
// 	
// 	std::cout << "Number of Hypothesis found " << H.size() << std::endl;
// 	
// 	cv::waitKey(0);
	
	
// 	std::cout << 
// 	"************************** SECOND TEST WITH DIFFERENT MAPS" 
// 	<<std::endl << "******************************************" << std::endl;
// 	
// 	std::deque< 
// 			std::pair < 
// 				AASS::graphmatch::VertexPlace, 
// 				AASS::graphmatch::VertexPlace > > places_pair_second;
// 	AASS::graphmatch::GraphPlace gp2_second = m2.getGraphPlace();
// 	AASS::graphmatch::GraphPlace gp_second = m.getGraphPlace();
// 	
// 	graphmatch.pairWiseMatch(gp_second, gp2_second, places_pair_second);
// 	
// 	std::deque< 
// 			std::pair < 
// 				AASS::graphmatch::VertexPlace, 
// 				AASS::graphmatch::VertexPlace > >::iterator it_second;
// 				
// 	for(it_second = places_pair_second.begin() ; it_second != places_pair_second.end(); it_second++){
// 		std::cout << "MATCH of room : " << gp_second.isRoom( (*it_second).first) << " with number of edge " << gp_second.getNumEdges( (*it_second).first) << std::endl;
// 		gp_second.print( (*it_second).first);
// 		gp2_second.print( (*it_second).second);
// 		
// 		BOOST_CHECK_EQUAL(gp_second.isRoom( (*it_second).first), gp2_second.isRoom( (*it_second).second));
// 		BOOST_CHECK_EQUAL(gp_second.getNumEdges( (*it_second).first), gp2_second.getNumEdges( (*it_second).second));
// 	
// 	}
// 	
// 	cv::waitKey(0);
// 	
// 	std::cout << "EXPANSION TEST" << std::endl;
// 	std::deque<std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > > > H2;
// 	
// 	graphmatch.growHypotheses(gp_second, gp2_second, H2);
// 	
// 	std::cout << "END RESULT" << std::endl;
// 	
// 	std::deque< std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > > >::iterator ittt_second;
// 	std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > >::iterator itt_second;
// 	
// 	cv::waitKey(0);
// 	
// 	for(ittt_second = H2.begin() ; ittt_second != H2.end() ; ittt_second++){
// 		
// 		std::cout << "Hypothesis" << std::endl << std::endl;
// 		cv::RNG rng(12345);
// 		cv::Mat temp = cv::Mat::zeros(mapgate.size(), CV_8UC1);
// 		cv::Mat hypoone = cv::Mat::zeros(mapgate.size(), CV_8UC1);
// 		cv::Mat hypotwo = cv::Mat::zeros(mapgate.size(), CV_8UC1);
// 		
// 		for(itt_second = (*ittt_second).begin() ; itt_second != (*ittt_second).end() ; itt_second++){
// 			cv::Scalar color;
// 			
// 			if(temp.channels() == 1){
// 				color = rng.uniform(50, 255);
// 			}
// 			
// 			else if(tmp.channels() == 3){
// 				color[1] = rng.uniform(50, 255);
// 				color[2] = rng.uniform(50, 255);
// 				color[3] = rng.uniform(50, 255);
// 			}
// 			
// 			std::cout <<"Match" << std::endl;
// 			gp_second.print( (*itt_second).first );
// 			gp_second.print( (*itt_second).second );
// 			
// 			gp_second.draw(temp, (*itt_second).first, color);
// 			gp_second.draw(temp, (*itt_second).second, color);
// 			
// 			cv::line(temp, gp_second[(*itt_second).first].mass_center, gp2_second[(*itt_second).second].mass_center, color);
// 			
// 		}
// 		itt_second = (*ittt_second).begin();
// 		itt_second++;
// 		cv::Scalar color;
// 			
// 		if(temp.channels() == 1){
// 			color = rng.uniform(50, 255);
// 		}
// 		
// 		else if(tmp.channels() == 3){
// 			color[1] = rng.uniform(50, 255);
// 			color[2] = rng.uniform(50, 255);
// 			color[3] = rng.uniform(50, 255);
// 		}
// 		
// 		for( ; itt_second != (*ittt_second).end() ; itt_second++){
// 			cv::line(hypoone, gp_second[(*(itt_second-1)).first].mass_center, gp_second[(*itt_second).first].mass_center, color);
// 			cv::line(hypotwo, gp2_second[(*(itt_second-1)).second].mass_center, gp2_second[(*itt_second).second].mass_center, color);
// 		}
// 		
// 		cv::imshow("hypo one", hypoone);
// 		cv::imshow("hypo two", hypotwo);
// 		cv::imshow("the hypo correspondence", temp);
// 		cv::waitKey(0);
// 		
// 	}
// 
// 	
// 	std::cout << "Number of Hypothesis found " << H2.size() << std::endl;
// 	
// 	cv::waitKey(0);
	
	
	
// 	std::cout << "THE REAL THING" << std::endl;
// 	
// 	
// 	AASS::graphmatch::GraphPlace gp_real;
// 	createGraph(gp_real);
// 	
// 	AASS::graphmatch::GraphPlace gp_real_model;
// 	createGraphtwo(gp_real_model);
// 	
// 	Thinker_Voronoi v_real;
// 	Thinker_Voronoi v2_real;
// 	Map m_real(v_real);
// 	Map m2_real(v2_real);
// 	
// 	m_real.importFile2System("../Test/TEST_COMPARISON/TEST2/map");
// 	m2_real.importFile2System("../Test/TEST_COMPARISON/TEST2/model");
// 
// 	cv::imshow("img", m_real.getObstacleMat());
// 	cv::imshow("img2", m2_real.getObstacleMat());
// 	m_real.think();
// 	m2_real.think();
// 	
// 	AASS::topologicalmap::GraphList glist;
// 	AASS::topologicalmap::GraphList glist2;
// 	
// 	glist = m_real.getGraphLines();
// 	glist.scale(2);
// 	glist2 = m2_real.getGraphLines();
// 	glist2.scale(2);
// 	
// 	graphmatch::GraphMatcher graphmatch_real(m_real, m2_real);
// 	
// 	int a;
// 	std::cin >> a;
// 	
// 	AASS::graphmatch::GraphPlace gp_from = m_real.getGraphPlace();
// 	AASS::graphmatch::GraphPlace gp_from2 = m2_real.getGraphPlace();
// 	
// 	graphmatch_real.planarEditDistanceAlgorithm(gp_from, gp_from2);
// 	
// 	std::cout << "OVER" << std::endl;
// 	
// 	std::deque<	
// 			std::pair<
// 				std::deque< 
// 					std::pair < 
// 						AASS::graphmatch::VertexPlace, 
// 						AASS::graphmatch::VertexPlace 
// 						> 
// 					>, 
// 				int 
// 			> 
// 		> hypothesis_final = graphmatch_real.getResult();
// 		
// 		
// 	std::deque< 
// 		std::pair < 
// 			AASS::graphmatch::VertexPlace, 
// 			AASS::graphmatch::VertexPlace 
// 			> 
// 		> list_result;
// 		
// 		
// 	int best =50;
// 	
// 	for(size_t i = 0 ; i < hypothesis_final.size() ; i++){
// 		if(best > hypothesis_final[i].second){
// 			best = hypothesis_final[i].second;
// 			list_result = hypothesis_final[i].first;
// 		}
// 	}
// 	
// 	BOOST_CHECK_EQUAL(best, 0);
// 	
// 	cv::Mat draw_links = cv::Mat::zeros(m_real.getObstacleMat().size(), CV_8UC3);
// 	cv::Mat draw_graph = cv::Mat::zeros(m_real.getObstacleMat().size(), CV_8UC3);
// 	cv::Mat draw_graph_model = cv::Mat::zeros(m_real.getObstacleMat().size(), CV_8UC3);
// 	
// 	cv::Scalar color;
// 	cv::RNG rrng(12345);
// 		
// 	if(draw_links.channels() == 1){
// 		color = rrng.uniform(50, 255);
// 	}
// 	
// 	else if(draw_links.channels() == 3){
// 		color[1] = rrng.uniform(50, 255);
// 		color[2] = rrng.uniform(50, 255);
// 		color[3] = rrng.uniform(50, 255);
// 	}
// 	
// 	cv::Scalar color_model;
// 		
// 	if(draw_links.channels() == 1){
// 		color_model = 100;
// 	}
// 	
// 	else if(draw_links.channels() == 3){
// 		color_model[1] = 100;
// 		color_model[2] = 100;
// 		color_model[3] = 100;
// 	}
// 	
// 	cv::Scalar color_one;
// 		
// 	if(draw_links.channels() == 1){
// 		color_one = 255;
// 	}
// 	
// 	else if(draw_links.channels() == 3){
// 		color_one[1] = 255;
// 		color_one[2] = 255;
// 		color_one[3] = 255;
// 	}
// 	
// 	
// 	for(size_t i = 0 ; i < list_result.size() ; i++ ){
// 		std::cout << "this : ";
// 		gp_from.print(list_result[i].first);
// 		std::cout << " linked to this : " ;
// 		gp_from2.print(list_result[i].second);
// 		std::cout << std::endl;
// 		
// 		cv::line(draw_links, gp_from[list_result[i].first].mass_center, gp_from2[list_result[i].second].mass_center, color);
// 		
// 		cv::circle(draw_links, gp_from2[list_result[i].second].mass_center, 10, color_model, 3);
// 		cv::circle(draw_links, gp_from[list_result[i].first].mass_center, 10, color_one, 3);
// 		
// 	}
// 	
// 	gp_from.draw(draw_graph);
// 	gp_from2.draw(draw_graph_model);
// 	
// 	cv::imshow("links", draw_links);
// 	cv::imshow("graph", draw_graph);
// 	cv::imshow("model", draw_graph_model);
// 	cv::waitKey(0);
	
}