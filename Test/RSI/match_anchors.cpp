#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
//#include <RSI/FuzzyOpening.hpp>
//#include <RSI/ZoneReducer.hpp>
//#include <RSI/ZoneExtractor.hpp>
#include <RSI/Uniqueness.hpp>

#include "RSIConversion.hpp"
#include "GraphMatcherAnchor.hpp"


#include "maoris/ZoneExtractor.hpp"
#include "maoris/FuzzyOpening.hpp"
//#include "maoris/Kmean.hpp"
#include "maoris/ZoneReducer.hpp"
#include "maoris/Segmentor.hpp"

#include "RSI/GraphZoneRI.hpp"
#include "RSI/ZoneCompared.hpp"
#include "RSI/hungarian/hungarian.h"
#include "RSI/HungarianMatcher.hpp"

void draw(AASS::RSI::GraphZoneRI& gp_real, AASS::RSI::GraphZoneRI& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, std::vector< AASS::RSI::ZoneCompared > matches){

	cv::Mat obst_copy;
	obstacle.copyTo(obst_copy);

	cv::Mat obst_model_copy;
	obstacle_model.copyTo(obst_model_copy);

// 	cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
// 	cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
// 	cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);

	int cols_max = obst_model_copy.size().width;
	if(cols_max < obst_copy.size().width){
		cols_max = obst_copy.size().width;
	}

	cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
	cv::Mat all = cv::Mat::zeros(size, CV_8UC3);
// 	cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
	cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);

	cv::Mat roi = all(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
// 	cv::Mat roi_linked = only_linked(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
	cv::Mat roi_model = all(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));

// 	cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
// 	cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));

// 	gp_real.draw(roi);
// 	gp_model.draw(roi_model);

	obst_copy.copyTo(roi);
	obst_model_copy.copyTo(roi_model);

	cv::Scalar color;
	cv::RNG rrng(12345);

	if(all.channels() == 1){
		color = rrng.uniform(50, 255);
	}

	else if(all.channels() == 3){
		color[1] = rrng.uniform(50, 255);
		color[2] = rrng.uniform(50, 255);
		color[3] = rrng.uniform(50, 255);
	}

	cv::Scalar color_model;


	auto it = matches.begin();

	for( ; it != matches.end() ; ++it){
		std::cout << "DRAW LINE " << std::endl;

		auto point = gp_model[it->target].getCentroid();
		point.y = point.y + obst_model_copy.size().height;

		cv::line(all, gp_real[it->source].getCentroid(), point, color, 5);
	}

	cv::imshow("all links", all);

}

//void makeGraphSLAM(const std::string& file, AASS::RSI::GraphZone& graph_slam){
//
//	cv::Mat slam_tmp = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
////
//// 	cv::imshow("input", slam_tmp);
//// 	cv::waitKey(0);
//
//	cv::threshold(slam_tmp, slam_tmp, 20, 255, cv::THRESH_BINARY);
//	cv::threshold(slam_tmp, slam_tmp, 20, 255, cv::THRESH_BINARY_INV);
//
//// 	cv::imshow("input", slam_tmp);
//// 	cv::waitKey(0);
//	std::cout << "/************ FUZZY OPENING*************/ \n";
//
//	cv::blur(slam_tmp, slam_tmp, cv::Size(15,15));
//
//// 	cv::imshow("input", slam_tmp);
//// 	cv::waitKey(0);
//
//	cv::threshold(slam_tmp, slam_tmp, 200, 255, cv::THRESH_BINARY);
//	cv::threshold(slam_tmp, slam_tmp, 20, 0, cv::THRESH_TOZERO);
//
//// 	cv::Mat invSrc =  cv::Scalar::all(255) - slam_tmp;
//
//// 	cv::imshow("input", slam_tmp);
//// 	cv::waitKey(0);
//
//	cv::Mat slam;
//	AASS::RSI::Kmeans kmeans;
//	kmeans.setK(4);
//	kmeans.kmeansColor(slam_tmp, slam);
//
//	AASS::RSI::FuzzyOpening fuzzy_slam;
//	fuzzy_slam.fast(false);
//
//	cv::Mat out_slam;
//// 	cv::imshow("SLAM", slam);
//// 	cv::waitKey(0);
//	fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
//	std::cout << "Done opening " << std::endl;
//	out_slam.convertTo(out_slam, CV_8U);
//
//// 	std::cout << out << std::endl;
//
//	std::cout << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
//	cv::Mat out_tmp_slam;
//	AASS::RSI::reduceZone(out_slam, out_tmp_slam);
//// 	cv::imshow("REDUCED", out_tmp_slam);
//// 	cv::waitKey(0);
//
//	AASS::RSI::ZoneExtractor zone_maker;
//	std::cout << "WHATERSHED SLAM" << std::endl;
//	zone_maker.extract(out_tmp_slam);
//
//	std::cout << "Got the ZONES" << std::endl;
//
//	// 	std::cout << "Getting the graph" << std::endl;
//
//	std::cout << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
//	graph_slam = zone_maker.getGraph();
//	graph_slam.removeVertexValue(0);
//
//	std::cout << "Number of nodes" << graph_slam.getNumVertices() << std::endl;
//
//	//Watershed Algorithm
//	graph_slam.watershed(0.25);
//
//	int size_to_remove = 100;
//	graph_slam.removeVertexUnderSize(size_to_remove, true);
//	graph_slam.removeLonelyVertices();
//	if(graph_slam.lonelyVertices())
//		throw std::runtime_error("Fuck you lonelyness");
//
//	cv::Mat graphmat2 = cv::Mat::zeros(out_tmp_slam.size(), CV_8U);
//	graph_slam.draw(graphmat2);
//	std::string s = "Blob";
//// 	cv::imshow(s, graphmat2);
//// 	cv::waitKey(0);
//}
//
//
//void makeGraph(const std::string& file, AASS::RSI::GraphZone& graph_slam){
//
//
//	cv::Mat slam = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
////
//// 	cv::imshow("input", slam);
//// 	cv::waitKey(0);
//
//	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
//	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);
//
//	std::cout << "/************ FUZZY OPENING*************/ \n";
//	AASS::RSI::FuzzyOpening fuzzy_slam;
//	fuzzy_slam.fast(false);
//
//	cv::Mat out_slam;
//// 	cv::imshow("SLAM", slam);
//// 	cv::waitKey(0);
//	fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
//	std::cout << "Done opening " << std::endl;
//	out_slam.convertTo(out_slam, CV_8U);
//
//// 	std::cout << out << std::endl;
//
//	std::cout << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
//	cv::Mat out_tmp_slam;
//	AASS::RSI::reduceZone(out_slam, out_tmp_slam);
//// 	cv::imshow("REDUCED", out_tmp_slam);
//// 	cv::waitKey(0);
//
//	AASS::RSI::ZoneExtractor zone_maker;
//	std::cout << "WHATERSHED SLAM" << std::endl;
//	zone_maker.extract(out_tmp_slam);
//
//	std::cout << "Got the ZONES" << std::endl;
//
//	// 	std::cout << "Getting the graph" << std::endl;
//// 	cv::imshow("trimming", out_tmp_slam);
//// 	cv::waitKey(0);
//	std::cout << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
//	graph_slam = zone_maker.getGraph();
//	graph_slam.removeVertexValue(0);
//
//	std::cout << "Number of nodes" << graph_slam.getNumVertices() << std::endl;
//
//// 	cv::imshow("watersherd", out_tmp_slam);
//// 	cv::waitKey(0);
//	//Watershed Algorithm
//	graph_slam.watershed(0.25);
//
//	int size_to_remove = 100;
//
//	graph_slam.removeVertexUnderSize(size_to_remove, true);
//	graph_slam.removeLonelyVertices();
//	if(graph_slam.lonelyVertices())
//		throw std::runtime_error("Fuck you lonelyness");
//
//	cv::Mat graphmat2 = cv::Mat::zeros(out_tmp_slam.size(), CV_8U);
//	graph_slam.draw(graphmat2);
//	cv::imshow("end", graphmat2);
//}


BOOST_AUTO_TEST_CASE(trying)
{
	
	
// 	bool res = re->compareKeypoints(re2);
// 	std::cout << "Same ? " << res << std::endl;
	
	/// TEST CONVERSION
	
// 	int argc = boost::unit_test::framework::master_test_suite().argc;
// 	char** argv = boost::unit_test::framework::master_test_suite().argv;
	std::string file;
	file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/model_simple.png";
	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);

//	AASS::RSI::GraphZone graph_slam;
// 	makeGraphSLAM(file, graph_slam);

	AASS::maoris::Segmentor segmenteur;
	AASS::maoris::GraphZone graph_segmented;

//	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	segmenteur.segmentImage(slam1, graph_segmented);
	cv::Mat segmented_map = segmenteur.getSegmentedMap();

	cv::imshow("Segmented", segmented_map);
	cv::waitKey(0);

	AASS::RSI::GraphZoneRI graph_slam(graph_segmented);
//	makeGraph(file, graph_slam);
	
	std::string file2;
	file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/03.png";
	cv::Mat slam2 = cv::imread(file2, CV_LOAD_IMAGE_GRAYSCALE);

//	AASS::RSI::GraphZone graph_slam2;
//	makeGraph(file2, graph_slam2);
	AASS::maoris::Segmentor segmenteur2;
	AASS::maoris::GraphZone graph_segmented2;

//	double time2 = 0;
// 	makeGraph(slam, graph_slam, time);
	segmenteur2.segmentImage(slam2, graph_segmented2);
	cv::Mat segmented_map2 = segmenteur2.getSegmentedMap();

	cv::imshow("Segmented", segmented_map2);
	cv::waitKey(0);

	AASS::RSI::GraphZoneRI graph_slam2(graph_segmented2);

	graph_slam.updatePCA();
	graph_slam.setPCAClassification();
	graph_slam.setSizesClassification();

	graph_slam2.updatePCA();
	graph_slam2.setPCAClassification();
	graph_slam2.setSizesClassification();

	
	
	/******** Calcul of uniqueness********************************************/
	
// 	/********** Uniqueness *******************************************/
// 	
// 	AASS::RSI::Uniqueness unique;
// 	
// 	std::cout << "FIRST UNIA" << std::endl;
// 	
// 	auto uni1 = unique.uniqueness(graph_slam);
// 	
// // 	/********** Uniqueness *******************************************/
// // 	
// 	std::cout << "SECOND UNIA" << std::endl;
// 	
// 	auto uni2 = unique.uniqueness(graph_slam2);
// 	
// 	assert(graph_slam.zoneUniquenessWasCalculated() == true);
// 	assert(graph_slam2.zoneUniquenessWasCalculated() == true);
	
//	graph_slam.setSDAwayFromMeanForUniqueness(1);
//	graph_slam2.setSDAwayFromMeanForUniqueness(1);
	
	/********** Uniqueness *******************************************/
//	graph_slam.updateUnique();
//	graph_slam2.updateUnique();
	
////	assert(graph_slam.zoneUniquenessWasCalculated() == true);
////	assert(graph_slam2.zoneUniquenessWasCalculated() == true);
//
//	cv::Mat gmatu = cv::Mat::zeros(slam1.size(), CV_8U);
//	graph_slam.drawUnique(gmatu);
//	cv::imshow("input unique", gmatu);
//
//	cv::Mat gmat2u = cv::Mat::zeros(slam2.size(), CV_8U);
//	graph_slam2.drawUnique(gmat2u);
//	cv::imshow("model unique", gmat2u);
//	cv::waitKey(0);
//
	
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
		std::dynamic_pointer_cast< AASS::graphmatch::ZoneKeypoint >(gp[v].getKeypoint())->zone.getMaxMinPCA();
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
	
	cv::imshow("TEST", slam1);
	draw(graph_slam, graph_slam2, slam1, slam2, match);
	cv::waitKey(0);
	
	
	/******** Matching *******************************************************/
	
	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "TESTING gp " << std::endl;
		std::dynamic_pointer_cast<  AASS::graphmatch::ZoneKeypoint >(gp[v].getKeypoint())->zone.getMaxMinPCA();
	}
	for (vp = boost::vertices(gp2.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "TESTING gp2 " << std::endl;
		std::dynamic_pointer_cast<   AASS::graphmatch::ZoneKeypoint >(gp2[v].getKeypoint())->zone.getMaxMinPCA();
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
	bool draw = false;
	graphmatcheranchor.anchorMatching(gp, gp2, anchors, draw, slam1.size() );
	std::deque<	AASS::graphmatch::Hypothese	> hypothesis_final = graphmatcheranchor.getResult();
	graphmatcheranchor.sort(hypothesis_final);
	
	old_score = score;
	score = hypothesis_final[0].getDist();
	
	hypothesis_final[0].drawHypo(gp, gp2, slam1, slam2, "finaleeee :D", 1);
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