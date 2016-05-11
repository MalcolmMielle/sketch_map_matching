#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "PlaceExtractorList2Place.hpp"
#include "PlaceExtractorRoomCorner.hpp"
#include "GraphPlace.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "vodigrex/linefollower/LineFollowerGraph.hpp"


void createGraph(AASS::graphmatch::GraphPlace& gp_test_editdistance){

	
	AASS::graphmatch::VertexPlace vnew_test_editdistance;
	AASS::graphmatch::VertexPlace v2new_test_editdistance;
	AASS::graphmatch::VertexPlace v1new_test_editdistance;
	AASS::graphmatch::VertexPlace v3new_test_editdistance;
	AASS::graphmatch::VertexPlace v4new_test_editdistance;
	AASS::graphmatch::VertexPlace v5new_test_editdistance;
	
	std::vector <cv::Point> conte_test_editdistance;
	cv::Point2i mcnew_test_editdistance(2, 2); 
	cv::Point2i mcnew1_test_editdistance(6, 2); //r    1
	cv::Point2i mcnew2_test_editdistance(2, 5); //c    4
	cv::Point2i mcnew3_test_editdistance(7, 2); //r    2
	cv::Point2i mcnew4_test_editdistance(3, 3); //r    3
	cv::Point2i mcnew5_test_editdistance(0, 0); //c    5
	
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
	
	AASS::graphmatch::EdgePlace enew_test_editdistance;
	AASS::graphmatch::EdgePlace enew2_test_editdistance;
	AASS::graphmatch::EdgePlace enew3_test_editdistance;

	AASS::graphmatch::Gateway gt_test_editdistance;

	gp_test_editdistance.addEdge(enew_test_editdistance, vnew_test_editdistance, v1new_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(enew2_test_editdistance,vnew_test_editdistance, v2new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v3new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v4new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v5new_test_editdistance,  gt_test_editdistance);
	
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


BOOST_AUTO_TEST_CASE(trying)
{

// 	AASS::topologicalmap::PlaceExtractor p;
// 	
// 	AASS::graphmatch::GraphPlace gp;
// 	
// 	AASS::graphmatch::VertexPlace v;
// 	AASS::graphmatch::VertexPlace v2;
// 	
// 	std::vector <cv::Point> cont;
// 	cv::Moments m;
// 	cv::Point2i mc;
// 	
// 	gp.addVertex(v, cont, m, mc);
// 	gp.addVertex(v2, cont, m ,mc);
// 	AASS::graphmatch::EdgePlace e;
// 	gp.addGateway(v, v2, cv::Point2i(1,1), cv::Point2i(2,2), e);
// 	
// 	BOOST_CHECK_EQUAL(gp.getGraph()[e].gateways.getFirst().x, 1);
// 	BOOST_CHECK_EQUAL(gp.getGraph()[e].gateways.getFirst().y, 1);
// 	BOOST_CHECK_EQUAL(gp.getGraph()[e].gateways.getSecond().y, 2);
// 	BOOST_CHECK_EQUAL(gp.getGraph()[e].gateways.getSecond().y, 2);
// 	
// 	BOOST_CHECK_EQUAL(gp.getNumVertices(), 2);
// 	BOOST_CHECK_EQUAL(gp.getNumEdges(), 1);
// 	gp.print();
// 	
// 	AASS::topologicalmap::Place ppp = gp[v2];
// 	ppp.mass_center.x = 3;
// 	BOOST_CHECK_EQUAL(ppp.mass_center.x, 3);
// 	BOOST_CHECK_EQUAL(ppp.mass_center.y, 0);
// 	
// 	AASS::graphmatch::Gateway g(cv::Point2i(0,3), cv::Point2i(0,-3) );
// 	
// 	BOOST_CHECK_EQUAL(g.isCrossingLine(cv::Point2i(12, 0), cv::Point2i(-12, 0)), true);
// 	BOOST_CHECK_EQUAL(g.isCrossingLine(cv::Point2i(12, 4), cv::Point2i(-12, 4)), false);
// 	BOOST_CHECK_EQUAL(g.isCrossingLine(cv::Point2i(1, 1), cv::Point2i(0, 1)), true);
// 	
	
	/****************** Graph list test *******/
	
// 	AASS::topologicalmap::GraphList gl;
// 	
// 	AASS::topologicalmap::Vertex vl;
// 	AASS::topologicalmap::Vertex vl1;
// 	AASS::topologicalmap::Vertex vl2;
// 	std::string type = "first";
// 	cv::Mat matt;
// 	cv::Point2i pt;
// 	
// 	gl.addVertex(type, matt, pt, vl);
// 	gl.addVertex(type, matt, pt, vl1);
// 	gl.addVertex(type, matt, pt, vl2);
// 	gl.addEdge(vl, vl1);
// 	gl.addEdge(vl, vl2);
// 	
// 	BOOST_CHECK_EQUAL(gl.getNumVertices(), 3);
// 	BOOST_CHECK_EQUAL(gl.getNumEdges(), 2);
// 	
// 	std::cout << "This is the graph " << std::endl;
// 	gl.print();
// 	
// 	std::deque<AASS::topologicalmap::Vertex> deq;
// 	gl.getAllVertexLinked(vl, deq);
// 	BOOST_CHECK_EQUAL(deq.size(), 2);
// 	
// 	for(size_t i = 0 ; i < deq.size(); i++){
// 		gl.print(deq[i]);
// 		std::cout << std::endl;
// 	}
// 	
// 	gl.addEdge(deq[0], deq[1]);
// 	
// 	BOOST_CHECK_EQUAL(gl.getNumEdges(), 3);
// 	
// 	gl.removeVertex(vl);
// 	
// 	BOOST_CHECK_EQUAL(gl.getNumVertices(), 2);
// 	BOOST_CHECK_EQUAL(gl.getNumEdges(), 1);
// 	
// 	gl.print();
	
	
	
	/**********************************/

// 	AASS::graphmatch::GraphPlace gpnew;
// 	
// 	AASS::graphmatch::VertexPlace vnew;
// 	AASS::graphmatch::VertexPlace v2new;
// 	AASS::graphmatch::VertexPlace v1new;
// 	AASS::graphmatch::VertexPlace v3new;
// 	AASS::graphmatch::VertexPlace v4new;
// 	AASS::graphmatch::VertexPlace v5new;
// 	
// 	std::vector <cv::Point> conte;
// 	cv::Point2i mcnew(2, 2);
// 	cv::Point2i mcnew2(2, 5);
// 	cv::Point2i mcnew1(6, 2);
// 	cv::Point2i mcnew3(7, 2);
// 	cv::Point2i mcnew4(3, 3);
// 	cv::Point2i mcnew5(0, 0);
// 	
// 	gpnew.addVertex(vnew, conte, m, mcnew);
// 	gpnew.addVertex(v1new, conte, m ,mcnew1);
// 	gpnew.addVertex(v2new, conte, m ,mcnew2);
// 	gpnew.addVertex(v3new, conte, m ,mcnew3);
// 	gpnew.addVertex(v4new, conte, m ,mcnew4);
// 	gpnew.addVertex(v5new, conte, m ,mcnew5);
// 	
// 	AASS::graphmatch::EdgePlace enew;
// 	AASS::graphmatch::EdgePlace enew2;
// 	AASS::graphmatch::EdgePlace enew3;
// 
// 	AASS::graphmatch::Gateway gt;
// 
// 	gpnew.addEdge(vnew, v1new, gt, enew);
// 	gpnew.addEdge(vnew, v2new, gt, enew2);
// 	gpnew.addEdge(vnew, v3new, gt, enew3);
// 	gpnew.addEdge(vnew, v4new, gt, enew3);
// 	gpnew.addEdge(vnew, v5new, gt, enew3);
// 	
// 	
// 	double angle = gp.angle(vnew, v1new, v2new);
// 	double angle2 = gp.angle(vnew, v2new, v1new);
// 	
// 	BOOST_CHECK(angle > 0);
// 	BOOST_CHECK(angle2 < 0);
// 	
// 	std::deque< std::pair< AASS::graphmatch::EdgePlace, AASS::graphmatch::VertexPlace > > all_edge;
// 	std::deque< std::pair< AASS::graphmatch::EdgePlace, AASS::graphmatch::VertexPlace > > all_edge_noorder;
// 	
// 	gpnew.getAllEdgeLinkedCounterClockWise(vnew, all_edge);
// 	gpnew.getAllEdgeLinked(vnew, all_edge_noorder);
// 	
// 	std::cout << "SIZE " << all_edge.size() << std::endl;
// 	BOOST_CHECK_EQUAL(all_edge.size(), 5);
// 	
// 	BOOST_CHECK(gpnew[all_edge[0].second].mass_center == gpnew[v1new].mass_center);
// 	BOOST_CHECK(gpnew[all_edge[1].second].mass_center == gpnew[v3new].mass_center);
// 	
// 	std::deque< std::pair< AASS::graphmatch::EdgePlace, AASS::graphmatch::VertexPlace > >::iterator it;
// 	for(it = all_edge.begin() ; it != all_edge.end() ; it++){
// 		gpnew.print((*it).second);
// 	}
// 	
// 	std::cout << "UNORDERED" << std::endl;
// 	
// 	for(it = all_edge_noorder.begin() ; it != all_edge_noorder.end() ; it++){
// 		gpnew.print((*it).second);
// 	}
// 
// 	AASS::topologicalmap::Vertex vert;
// 	AASS::topologicalmap::Intersection_Graph inter;
// 	gpnew.addCrossing(all_edge[1].second, vert, inter );
// 	
// 	std::string string("string");
// 	std::string string1("string");
// 	std::string string2("string2");
// 	
// 	std::string outt;
// 	BOOST_CHECK_EQUAL(levenshtein_distance(string, string1, outt), 0);
// 	BOOST_CHECK_EQUAL(levenshtein_distance(string, string2, outt), 1);
// 	
// 	
// 	std::string string_place;
// 	for(it = all_edge.begin() ; it != all_edge.end() ; it++){
// 		if(gpnew.isRoom((*it).second) == true){
// 			string_place = string_place + "r";
// 		}
// 		else{
// 			string_place = string_place + "c";
// 		}
// 	}
// 		
// 	BOOST_CHECK_EQUAL(string_place, "rcrrr");
// 	
// 	
// 	gpnew.addCrossing(all_edge[3].second, vert, inter );
// 	
// 	std::string string_place2;
// 	for(it = all_edge.begin() ; it != all_edge.end() ; it++){
// 		if(gpnew.isRoom((*it).second) == true){
// 			string_place2 = string_place2 + "r";
// 		}
// 		else{
// 			string_place2 = string_place2 + "c";
// 		}
// 	}
// 		
// 	BOOST_CHECK_EQUAL(string_place2, "rcrcr");
// 	
// 	std::deque< std::pair< AASS::graphmatch::VertexPlace, AASS::graphmatch::VertexPlace > > out;
// 	int dist = gpnew.editDistance(vnew, "rcccr", all_edge, out);
// 	
// 	int dist2 = gpnew.editDistance(vnew, "c", all_edge, out);
// 	
// 	BOOST_CHECK_EQUAL(dist, 1);
// 	BOOST_CHECK_EQUAL(dist2, 4);
// 	
// 	int d = levenshtein_distance("democrat", "republican", outt);
// 	
// 	BOOST_CHECK_EQUAL(outt.size(), d + 2);
// 	
// 	std::cout << "operation . " << outt << std::endl;
// 	
// 	d = levenshtein_distance( "republican", "democrat",outt);
// 	
// 	std::cout << "operation . " << outt << std::endl;
// 	
// 	int a;
// 	std::cout << "Pause enter a number to continue" << std::endl;
// 	std::cin >> a;
// 	
// 	
	
	
	AASS::graphmatch::GraphPlace gp_test_editdistance;
	
	AASS::graphmatch::VertexPlace vnew_test_editdistance;
	AASS::graphmatch::VertexPlace v2new_test_editdistance;
	AASS::graphmatch::VertexPlace v1new_test_editdistance;
	AASS::graphmatch::VertexPlace v3new_test_editdistance;
	AASS::graphmatch::VertexPlace v4new_test_editdistance;
	AASS::graphmatch::VertexPlace v5new_test_editdistance;
	
	std::vector <cv::Point> conte_test_editdistance;
	cv::Point2i mcnew_test_editdistance(2, 2); 
	cv::Point2i mcnew1_test_editdistance(6, 2); //r    1
	cv::Point2i mcnew2_test_editdistance(2, 5); //c    4
	cv::Point2i mcnew3_test_editdistance(7, 2); //r    2
	cv::Point2i mcnew4_test_editdistance(3, 3); //r    3
	cv::Point2i mcnew5_test_editdistance(0, 0); //c    5
	
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
	
	AASS::graphmatch::EdgePlace enew_test_editdistance;
	AASS::graphmatch::EdgePlace enew2_test_editdistance;
	AASS::graphmatch::EdgePlace enew3_test_editdistance;

	AASS::graphmatch::Gateway gt_test_editdistance;

	gp_test_editdistance.addEdge(enew_test_editdistance, vnew_test_editdistance, v1new_test_editdistance, gt_test_editdistance);
	gp_test_editdistance.addEdge(enew2_test_editdistance,vnew_test_editdistance, v2new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v3new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v4new_test_editdistance,  gt_test_editdistance);
	gp_test_editdistance.addEdge(enew3_test_editdistance,vnew_test_editdistance, v5new_test_editdistance,  gt_test_editdistance);
	
	
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
	
	gp_test_editdistance.print();
	
	
	std::deque< std::pair< AASS::graphmatch::EdgePlace, AASS::graphmatch::VertexPlace > > all_edge_test_editdistance;
	
	gp_test_editdistance.getAllEdgeLinkedCounterClockWise(vnew_test_editdistance, all_edge_test_editdistance);

	//Make a string out of the order of vertex.	
	std::string string_test_editdistance = gp_test_editdistance.makeString(all_edge_test_editdistance);
	
	std::deque< AASS::graphmatch::Match > out_test_editdistance;
	
	std::string op;
	int dist_test_editdistance = gp_test_editdistance.editDistance(vnew_test_editdistance, string_test_editdistance, all_edge_test_editdistance, out_test_editdistance, op);
	
	BOOST_CHECK_EQUAL(dist_test_editdistance, 0);
	BOOST_CHECK_EQUAL(out_test_editdistance.size(), 5);
	
	for(size_t i = 0 ; i < out_test_editdistance.size() ; i++ ){
		std::cout << "this : ";
		gp_test_editdistance.print(out_test_editdistance[i].getFirst() );
		std::cout << " linked to this : " ;
		gp_test_editdistance.print(out_test_editdistance[i].getSecond() );
		std::cout << std::endl;
	}
	
	int b;
	std::cout << "Pause enter a number to continue" << std::endl;
	std::cin >> b;	
	
	
// 	cv::Mat map = cv::imread("../Test/ObstacleMapBug.png");
// 	cv::imshow("result", map);
// 	cv::waitKey(0);
// 	Thinker_Voronoi tv;
// 	tv.modeLaplaceVoro();
// 	tv.setDownSample(1);
// 	AASS::topologicalmap::LineFollower lf;
// 	
// 	tv.think(map);
// 	cv::Mat voronoi = tv.getResult();
// 	cv::imshow("result", voronoi);
// 	cv::waitKey(0);
// 	
// 	lf.inputMap(voronoi);
// 	lf.thin();
// 	
// 	cv::imshow("result", lf.getResult());
// 	cv::waitKey(0);
// 	
// 	p.inputMapIn(map);
// 	p.inputLines(lf.getResult());
// 	p.inputDistance(voronoi);
//  	p.inputPreviousInfo(lf.getGraph());
// 	
// 	p.init();
// 	p.drawCircle();
// 	
// 	cv::imshow("ending", p.getResult());
// 	cv::waitKey(0);
// 	
	
// 	AASS::topologicalmap::PlaceExtractor pp;
// 	AASS::topologicalmap::PlaceExtractor ppp;
// 	
// 	pp.setRoiCenter(cv::Point2d(50, 50));
// 	ppp.setRoiCenter(cv::Point2d(99/2, 99/2));
// 	pp.setRoiRadius(45);
// 	ppp.setRoiRadius(43);
// 	cv::Mat minin = cv::imread("../Test/100x100.png");
// 	cv::Mat minin2 = cv::imread("../Test/99x99.png");
// 	pp.inputMapIn(minin);
// 	ppp.inputMapIn(minin2);
// 	cv::Mat out;
// 	pp.keepCircle(out);
// 	
// // 	std::cout << "OUUUUUT : " <<  out << std::endl;
// 	
// 	cv::Mat gate = cv::imread("../Test/gateway.png", CV_LOAD_IMAGE_GRAYSCALE);
// 	std::vector<cv::Point2i> points;
// // 	pp.detectGateway(gate, points);
// 	
// 	cv::Mat blob = cv::imread("../Test/realblob_modified.png", CV_LOAD_IMAGE_GRAYSCALE);
// 	cv::Mat blob_color = cv::imread("../Test/realblob_modified.png");
// 	cv::Mat invSrc;
// // 	cv::copyMakeBorder(blob, invSrc, 10, 10, 10, 10, cv::BORDER_CONSTANT, 0);
// 	invSrc =  cv::Scalar::all(255) - blob;
// // 	cv::Mat invSrc = blob;
// 	cv::normalize(invSrc, invSrc, 0, 255, cv::NORM_MINMAX);
// 	cv::threshold(invSrc, invSrc, 50, 255, CV_THRESH_BINARY);
// 	cv::imshow("inv image is : ", invSrc);
// 	cv::waitKey(0);
// 	std::vector< cv::Point2i > all_points;
// 	
// 	std::cout << "size : " << blob.size().width /2 ;
// 	pp.setRoiRadius(blob.size().width /2);
// 	pp.setRoiCenter(cv::Point2d(blob.size().width /2, blob.size().height /2));
// 
// 	pp.inputMapIn(blob_color);
// 	pp.detectGatewayBlob(invSrc, all_points);
// 	
// 	pp.drawGateways();
// 	
// 	cv::imshow("result", pp.getResult());
// 	cv::waitKey(0);
	
// 	for(size_t i = 0 ; i < points.size() ; i++){
// 		std::cout << "points : " << points[i] << std::endl;
// 	}
	
// 	BOOST_CHECK_EQUAL(points[0].x, 2);
// 	BOOST_CHECK_EQUAL(points[0].y, 0);
// 	BOOST_CHECK_EQUAL(points[1].x, 9);
// 	BOOST_CHECK_EQUAL(points[1].y, 0);
// 	BOOST_CHECK_EQUAL(points[2].x, 9);
// 	BOOST_CHECK_EQUAL(points[2].y, 0);
// 	BOOST_CHECK_EQUAL(points[3].x, 19);
// 	BOOST_CHECK_EQUAL(points[3].y, 15);
// 	BOOST_CHECK_EQUAL(points[4].x, 19);
// 	BOOST_CHECK_EQUAL(points[4].y, 19);
// 	BOOST_CHECK_EQUAL(points[5].x, 0);
// 	BOOST_CHECK_EQUAL(points[5].y, 12);
	
	
	int level;
	std::cout << "set level" << std::endl;
	std::cin >> level;
	
	int marge;
	std::cout << "min size of waals in pixel" << std::endl;
	std::cin >> marge;
	
	for(int mode = 0 ; mode < 3 ; mode++){
	
		AASS::graphmatch::PlaceExtractorRoomCorner placeex;
		
		placeex.setMode(mode);
		
		AASS::vodigrex::ThinkerVoronoi tvv;
		tvv.modeLaplaceVoro();
		tvv.setDownSample(1);
		
		cv::Mat mapgate = cv::imread("../Test/realmap.png");
		
		cv::threshold(mapgate, mapgate, 150, 255, CV_THRESH_BINARY_INV);
		
		cv::imshow("scene", mapgate);
	// 	cv::waitKey(0);
		
		
		
		tvv.setLevel(level);
		tvv.think(mapgate);
		
		
		AASS::vodigrex::LineFollowerGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> l;
		l.setMarge(marge);
		l.inputMap(tvv.getResult());
		l.thin();
		
		cv::Mat tmp = cv::Mat::zeros(mapgate.size(), CV_8UC3);
		AASS::topologicalmap::GraphLine ggg(l.getGraph());
		ggg.scale(2);
		ggg.draw(tmp);
		
		ggg.print();
		
		cv::imshow("old graph", tmp);
		cv::imshow("resulet", tvv.getResult());
		cv::imshow("scene", mapgate);
		cv::waitKey(0);
		
		placeex.inputMapIn(mapgate);
		AASS::topologicalmap::GraphLine graphlist(l.getGraph() );
		graphlist.scale(2);
		placeex.inputGraph(graphlist);
		placeex.init();
		
		AASS::topologicalmap::GraphLine gpruned( l.getGraph() );
		gpruned.scale(2);
// 		placeex.prunePreviousGraph();
		
		
		placeex.extract();
		
		gpruned.print();
		
		cv::Mat end = cv::Mat::zeros( mapgate.size(), CV_8UC1);
		cv::Mat end_2 = cv::Mat::zeros( mapgate.size(), CV_8UC1);
		
		gpruned.draw(end);
		
		
		AASS::topologicalmap::GraphLine gresult( l.getGraph() );
		gresult.scale(2);
		gresult.draw(end_2);
		
		placeex.printGraph();
		
		
		cv::Mat clone = mapgate.clone();
		AASS::graphmatch::GraphPlace places = placeex.getGraph();
		places.draw(clone);
		
		
		cv::imshow("result", placeex.getResult());
		cv::imshow("lines", l.getResult());
		cv::imshow("Old graph", end_2);
		cv::imshow("New graph", end);
		cv::imshow("in the end", clone);
		cv::waitKey(0);
		
		
		
			
		
		/**** Obstacle crossing ***/
		
	
	}
	
	
}