#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
#include <RSI/FuzzyOpening.hpp>
#include <RSI/ZoneReducer.hpp>
#include <RSI/ZoneExtractor.hpp>
#include <RSI/Uniqueness.hpp>

#include "RSIConversion.hpp"
#include "GraphMatcherAnchor.hpp"


void makeGraph(const std::string& file, AASS::RSI::GraphZone& graph_slam){
	
	
	cv::Mat slam = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
// 	
// 	cv::imshow("input", slam);
// 	cv::waitKey(0);
	
	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);
	
	std::cout << "/************ FUZZY OPENING*************/ \n";
	AASS::RSI::FuzzyOpening fuzzy_slam;
	fuzzy_slam.fast(false);
	
	cv::Mat out_slam;
// 	cv::imshow("SLAM", slam);
// 	cv::waitKey(0);
	fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
	std::cout << "Done opening " << std::endl;
	out_slam.convertTo(out_slam, CV_8U);
	
// 	std::cout << out << std::endl;
	
	std::cout << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
	cv::Mat out_tmp_slam;
	AASS::RSI::reduceZone(out_slam, out_tmp_slam);
// 	cv::imshow("REDUCED", out_tmp_slam);
// 	cv::waitKey(0);
	
	AASS::RSI::ZoneExtractor zone_maker;
	std::cout << "WHATERSHED SLAM" << std::endl;
	zone_maker.extract(out_tmp_slam);
	
	std::cout << "Got the ZONES" << std::endl;

	// 	std::cout << "Getting the graph" << std::endl;
	
	std::cout << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
	graph_slam = zone_maker.getGraph();
	graph_slam.removeVertexValue(0);

	std::cout << "Number of nodes" << graph_slam.getNumVertices() << std::endl;
	
	//Watershed Algorithm
	graph_slam.watershed(0.25);
	
	int size_to_remove = 100;
	graph_slam.removeVertexUnderSize(size_to_remove, true);
	graph_slam.removeLonelyVertices();
	if(graph_slam.lonelyVertices())
		throw std::runtime_error("Fuck you lonelyness");	
	
	cv::Mat graphmat2 = cv::Mat::zeros(out_tmp_slam.size(), CV_8U);
	graph_slam.draw(graphmat2);
	cv::imshow("end", graphmat2);
}


BOOST_AUTO_TEST_CASE(trying)
{
	
	
// 	bool res = re->compareKeypoints(re2);
// 	std::cout << "Same ? " << res << std::endl;
	
	/// TEST CONVERSION
	
// 	int argc = boost::unit_test::framework::master_test_suite().argc;
// 	char** argv = boost::unit_test::framework::master_test_suite().argv;
	std::string file;
	file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/00.png";
	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);

	AASS::RSI::GraphZone graph_slam;
	makeGraph(file, graph_slam);
	
	std::string file2;
	file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/01.png";
	cv::Mat slam2 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);

	AASS::RSI::GraphZone graph_slam2;
	makeGraph(file2, graph_slam2);
		
	
	/********** PCA of all zones in Graph and removing the ripples **********/
	
// 	graph_slam.updatePCA();
// 	graph_slam.removeRiplesv2();
// 	graph_slam.updateContours();
	graph_slam.update();
	
	graph_slam2.update();
	
	std::cout << "Size of graph" << graph_slam.getNumVertices() << std::endl;

	
// 	std::cout << "Size of graph2" << graph_slam2.getNumVertices() << std::endl;
	
	
	/******** Calcul of uniqueness********************************************/
	
	/********** Uniqueness *******************************************/
	
	AASS::RSI::Uniqueness unique;
	
	std::cout << "FIRST UNIA" << std::endl;
	
	auto uni1 = unique.uniqueness(graph_slam);
	
// 	/********** Uniqueness *******************************************/
// 	
	std::cout << "SECOND UNIA" << std::endl;
	
	auto uni2 = unique.uniqueness(graph_slam2);
	
	assert(graph_slam.zoneUniquenessWasCalculated() == true);
	assert(graph_slam2.zoneUniquenessWasCalculated() == true);
	
	
	/********** Hungarian matching of graph onto itself***************/
			
	std::cout << "Hungarian Match" << std::endl;
	AASS::RSI::HungarianMatcher hungmatch;
	std::vector<int> scores;
	auto match = hungmatch.match(graph_slam, graph_slam2, scores);
	
// 	exit(0);
	
	std::sort(match.begin(), match.end(), [&graph_slam, &graph_slam2](AASS::RSI::ZoneCompared &match, AASS::RSI::ZoneCompared &match1){
		return match.getRanking(graph_slam, graph_slam2) > match1.getRanking(graph_slam, graph_slam2); 
	} );
	
	/********** Drawing the graphs *******************************************/
	
	AASS::graphmatch::GraphPlace gp;
	AASS::graphmatch::RSIGraphConverter converter;
	converter.graphZoneToGraphPlace(graph_slam, gp);
	
	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "TESTING gp after construction" << std::endl;
		auto outt = std::dynamic_pointer_cast< AASS::graphmatch::ZoneKeypoint >(gp[v].getKeypoint())->zone.getMaxMinPCA();		
	}
	
	AASS::graphmatch::GraphPlace gp2;
	AASS::graphmatch::RSIGraphConverter converter2;
	converter2.graphZoneToGraphPlace(graph_slam2, gp2);
	
	cv::Mat m = cv::Mat::zeros(slam1.size(), CV_8U);
	gp.draw(m);
	cv::imshow("graph1 m", m);
	
	cv::Mat m2 = cv::Mat::zeros(slam2.size(), CV_8U);
	gp2.draw(m2);
	cv::imshow("graph2 m", m);
	cv::waitKey(0);
	
	
	/******** Matching *******************************************************/
	
	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "TESTING gp " << std::endl;
		auto outt = std::dynamic_pointer_cast<  AASS::graphmatch::ZoneKeypoint >(gp[v].getKeypoint())->zone.getMaxMinPCA();		
	}
	for (vp = boost::vertices(gp2.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "TESTING gp2 " << std::endl;
		auto outt = std::dynamic_pointer_cast<   AASS::graphmatch::ZoneKeypoint >(gp2[v].getKeypoint())->zone.getMaxMinPCA();		
	}
	
	
	
	AASS::graphmatch::GraphMatcherAnchor graphmatcheranchor;
// 	AASS::graphmatch::GraphMatcherClusterFiltered graphmatchold;
	
	double score = -1;
	double old_score = -1;
	int index_anchor = 0;
	
// 	auto vertex_anchor1 = match[0].source;
// 	auto vertex_anchor2 = match[0].target;
	
	std::deque < AASS::graphmatch::Match > anchors;

	for(size_t i = 0 ; i < match.size() ; ++i){
	
		auto vertex_anchor1 = match[i].source;
		auto vertex_anchor2 = match[i].target;
		
		AASS::graphmatch::VertexPlace vertex_place_anchor_source;
		bool found = converter.getEquivalentVPlace(vertex_anchor1, vertex_place_anchor_source);
		AASS::graphmatch::VertexPlace vertex_place_anchor_target;
		bool found2 = converter2.getEquivalentVPlace(vertex_anchor2, vertex_place_anchor_target);
		
		assert(found == true);
		assert(found2 == true);
		
		AASS::graphmatch::Match match_p(vertex_place_anchor_source, vertex_place_anchor_target);
		anchors.push_back(match_p);
	}
	
	
// 	while (old_score == -1 || score < old_score){
	++index_anchor;
	//MY THING
	bool draw = true;
	graphmatcheranchor.anchorMatching(gp, gp2, anchors, draw, slam1.size() );
	std::deque<	AASS::graphmatch::Hypothese	> hypothesis_final = graphmatcheranchor.getResult();
	graphmatcheranchor.sort(hypothesis_final);
	
	old_score = score;
	score = hypothesis_final[0].getDist();
	
	hypothesis_final[0].drawHypo(gp, gp2, slam1, slam2, "finaleeee", 1);
	cv::waitKey(0);
	
	std::cout << "Score " << score << " old_score " << old_score << std::endl; 
		
// 		vertex_anchor1 = match[index_anchor].source;
// 		vertex_anchor2 = match[index_anchor].target;
		
// 		converter.getEquivalentVPlace(vertex_anchor1, vertex_place_anchor_source);
// 		converter2.getEquivalentVPlace(vertex_anchor2, vertex_place_anchor_target);
// 		AASS::graphmatch::Match match_tmp(vertex_place_anchor_source, vertex_place_anchor_target);
// 		anchors.push_back(match_tmp);
// 	}
	
}