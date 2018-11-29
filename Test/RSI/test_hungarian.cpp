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

#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"






void draw(AASS::RSI::GraphZoneRI& gp_real, AASS::RSI::GraphZoneRI& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, std::vector< AASS::RSI::ZoneCompared > matches){

	cv::Mat obst_copy;
	obstacle.copyTo(obst_copy);

	cv::imshow("obstacle copy", obst_copy);
	cv::imshow("obstacle", obstacle);

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
	cv::Mat all = cv::Mat::zeros(size, CV_8UC1);
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
	cv::imshow("roi", roi);
	cv::imshow("alltmp", all);
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

void drawLaplacian(AASS::graphmatch::GraphLaplacian& gp_real, AASS::graphmatch::GraphLaplacian& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, std::vector< AASS::graphmatch::MatchLaplacian > matches){

	cv::Mat obst_copy;
	obstacle.copyTo(obst_copy);
//	cv::Mat obst_copy;
//	obst_copy2.convertTo(obst_copy, CV_8UC3);

	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
	//vertices access all the vertix
	for (vp = boost::vertices(gp_real); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		double value = gp_real[v].getHeat();
		gp_real[v].zone.drawZone(obst_copy, cv::Scalar(value * 255) );
	}

	cv::imshow("OBST COLOR", obst_copy);


	cv::Mat obst_model_copy;
	obstacle_model.copyTo(obst_model_copy);

	//vertices access all the vertix
	for (vp = boost::vertices(gp_model); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
		double value = gp_model[v].getHeat();
		gp_model[v].zone.drawZone(obst_model_copy, cv::Scalar(value * 255) );
	}

// 	cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
// 	cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
// 	cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);

	int cols_max = obst_model_copy.size().width;
	if(cols_max < obst_copy.size().width){
		cols_max = obst_copy.size().width;
	}

	cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
	cv::Mat all = cv::Mat::zeros(size, CV_8UC1);
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

		auto point = gp_model[it->getSecond()].getCenter();
		point.y = point.y + obst_copy.size().height;

		cv::line(all, gp_real[it->getFirst()].getCenter(), point, color, 5);

		auto point2 = gp_real[it->getFirst()].getCenter();

		std::string text;
		text = std::to_string(it->getCost());
		cv::putText(all, text, (point + point2)/2, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255));

	}

	cv::imshow("all links Laplacian", all);

}

void seeHungarian(const std::vector< AASS::RSI::ZoneCompared >& match, AASS::RSI::GraphZoneRI graph_slam, AASS::RSI::GraphZoneRI graph_slam2, cv::Mat& slam1, cv::Mat& slam2){
	//
// 	/********** Visualization ****************************************/
//
	for(size_t i = 0 ; i < match.size() ; ++i){
		std::cout << "matching " << i << " : " << match[i].source << " " << match[i].target << std::endl;
		cv::imshow("Zone1", graph_slam[match[i].source].getZoneMat());
		cv::imshow("Zone2", graph_slam2[match[i].target].getZoneMat());

		//TODO: Add uniqueness measurement with it
		std::cout << "SCORE of similarity (diff than uniqueness), it's the matching score between the zones, 0 is good, 1 is bad : " <<  " \nUniqueness : ";

		std::cout << graph_slam[match[i].source].getUniquenessScore() << " ";
		std::cout << graph_slam2[match[i].target].getUniquenessScore() << " ";
		std::cout << graph_slam[match[i].source].getUniquenessScore() + graph_slam2[match[i].target].getUniquenessScore() << " ";

		std::cout << "Match print :" << std::endl;

		match[i].print();

		std::cout << "\nrank/similarity " << match[i].getSimilarity() << std::endl;

		std::cout << std::endl << "zone 1 ";
		graph_slam[match[i].source].print();
		std::cout << std::endl << "zone 2 ";
		graph_slam2[match[i].target].print();

// 		for( auto it = uni1.begin(); it != uni1.end() ; ++it){
// 			if(it->first == match[i].first){
// 				std::cout << it->second << " ";
// 			}
// 		}
// 		std::cout << " And " ;
// 		for( auto it = uni2.begin(); it != uni2.end() ; ++it){
// 			if(it->first == match[i].second){
// 				std::cout << it->second << " ";
// 			}
// 		}
//
		std::cout << std::endl;

		cv::waitKey(0);
	}

	cv::imshow("TEST", slam1);
	draw(graph_slam, graph_slam2, slam1, slam2, match);
	cv::waitKey(0);
}


void seeHungarianLaplacian(const std::vector< AASS::graphmatch::MatchLaplacian >& match, AASS::graphmatch::GraphLaplacian& graph_slam, AASS::graphmatch::GraphLaplacian& graph_slam2, cv::Mat& slam1, cv::Mat& slam2){
	//
// 	/********** Visualization ****************************************/
//
//	for(size_t i = 0 ; i < match.size() ; ++i){
//		std::cout << "matching " << i << " : " << match[i].getFirst() << " " << match[i].getSecond() << std::endl;
//		cv::imshow("Zone1", graph_slam[match[i].getFirst()].getZoneMat());
//		cv::imshow("Zone2", graph_slam2[match[i].getSecond()].getZoneMat());
//
//		//TODO: Add uniqueness measurement with it
//		std::cout << "SCORE of similarity (diff than uniqueness), it's the matching score between the zones, 0 is good, 1 is bad : " <<  " \nUniqueness : ";
//
//		std::cout << graph_slam[match[i].getFirst()].getUniqueness() << " ";
//		std::cout << graph_slam2[match[i].getSecond()].getUniqueness() << " ";
//		std::cout << graph_slam[match[i].getFirst()].getUniqueness() + graph_slam2[match[i].getSecond()].getUniqueness() << " ";
//
////		std::cout << "Match print :" << std::endl;
//
////		match[i].print();
//
//		std::cout << "\nrank/similarity " << match[i].getCost() << std::endl;
//
////		std::cout << std::endl << "zone 1 ";
////		graph_slam[match[i].getFirst()].print();
////		std::cout << std::endl << "zone 2 ";
////		graph_slam2[match[i].target].print();
//
//// 		for( auto it = uni1.begin(); it != uni1.end() ; ++it){
//// 			if(it->first == match[i].first){
//// 				std::cout << it->second << " ";
//// 			}
//// 		}
//// 		std::cout << " And " ;
//// 		for( auto it = uni2.begin(); it != uni2.end() ; ++it){
//// 			if(it->first == match[i].second){
//// 				std::cout << it->second << " ";
//// 			}
//// 		}
////
//		std::cout << std::endl;
//
//		cv::waitKey(0);
//	}

	cv::imshow("TEST", slam1);
	drawLaplacian(graph_slam, graph_slam2, slam1, slam2, match);
	cv::waitKey(0);
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


cv::Mat makeGraph(const std::string& file, AASS::RSI::GraphZoneRI& graph_slam){

	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
/** Segmenting the map**/
	AASS::maoris::Segmentor segmenteur;
	AASS::maoris::GraphZone graph_segmented;

	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	time = segmenteur.segmentImage(slam1, graph_segmented);
	cv::Mat segmented_map = segmenteur.getSegmentedMap();

	cv::imshow("Segmented", segmented_map);
	cv::waitKey(0);

	graph_slam = AASS::RSI::GraphZoneRI(graph_segmented);

	graph_slam.updatePCA();
	graph_slam.setPCAClassification();
	graph_slam.setSizesClassification();

	return segmented_map;
}








BOOST_AUTO_TEST_CASE(trying)
{
	bool is_sketch = true;
	int argc = boost::unit_test::framework::master_test_suite().argc;
	char** argv = boost::unit_test::framework::master_test_suite().argv;

	std::string file;
	if(argc > 1){
		file = argv[1];
	}
	else{
		file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/01.png";
	}

	std::string file2;
	if(argc > 2){
		file2 = argv[2];
	}
	else{
		file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/model_simple.png";
	}

	if(argc > 3){
		std::stringstream ss(argv[3]);
		if(!(ss >> std::boolalpha >> is_sketch)) {
			std::cout << "NOT CORRECT BOOLEAN VALUE" << std::endl;
		}
	}
	if(is_sketch) std::cout << "Is a SKETCH" << std::endl;
	else std::cout << "Is NOT a sketch" << std::endl;

	AASS::RSI::GraphZoneRI graph_slam;
	cv::Mat graph_slam_segmented = makeGraph(file, graph_slam);

	AASS::RSI::GraphZoneRI graph_slam2;
	cv::Mat graph_slam2_segmented = makeGraph(file2, graph_slam2);


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
	/********** Uniqueness *******************************************/
	graph_slam.setSDAwayFromMeanForUniqueness(1);
	graph_slam2.setSDAwayFromMeanForUniqueness(1);

	/********** Uniqueness *******************************************/
	graph_slam.updateUnique();
	if(is_sketch) {
		graph_slam2.updateUnique();
	}
	else{
		std::cout << "IS NOT A SKETCH" << std::endl;
		graph_slam2.updateUnique(graph_slam);
	}

	assert(graph_slam.zoneUniquenessWasCalculated() == true);
	assert(graph_slam2.zoneUniquenessWasCalculated() == true);

	cv::Mat gmatu = cv::Mat::zeros(graph_slam_segmented.size(), CV_8U);
	graph_slam.drawUnique(gmatu);
	cv::imshow("input unique", gmatu);

	cv::Mat gmat2u = cv::Mat::zeros(graph_slam2_segmented.size(), CV_8U);
	graph_slam2.drawUnique(gmat2u);
	cv::imshow("model unique", gmat2u);
	cv::waitKey(0);

	/********** Hungarian matching of graph onto itself***************/

	std::cout << "Hungarian Match" << std::endl;
	AASS::RSI::HungarianMatcher hungmatch;
	std::vector<int> scores;
	auto match = hungmatch.match(graph_slam, graph_slam2, scores);

// 	exit(0);

	std::sort(match.begin(), match.end(), [&graph_slam, &graph_slam2](AASS::RSI::ZoneCompared &match, AASS::RSI::ZoneCompared &match1){
		return match.getSimilarity() < match1.getSimilarity();
	} );


	seeHungarian(match, graph_slam, graph_slam2, graph_slam_segmented, graph_slam2_segmented);


	/********** Drawing the graphs *******************************************/

	AASS::graphmatch::GraphPlace gp;
	AASS::graphmatch::GraphLaplacian gp_laplacian;
	AASS::graphmatch::RSIGraphConverter converter;
	converter.graphZoneToGraphPlace(graph_slam, gp);
	converter.graphZonetoGraphLaplacian(graph_slam, gp_laplacian);

	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::VertexPlace v = *vp.first;
		std::cout << "TESTING gp after construction" << std::endl;
		std::dynamic_pointer_cast< AASS::graphmatch::ZoneKeypoint >(gp[v].getKeypoint())->zone.getMaxMinPCA();
	}

	AASS::graphmatch::GraphPlace gp2;
	AASS::graphmatch::GraphLaplacian gp2_laplacian;
	AASS::graphmatch::RSIGraphConverter converter2;
	converter2.graphZoneToGraphPlace(graph_slam2, gp2);
	converter2.graphZonetoGraphLaplacian(graph_slam2, gp2_laplacian);

	cv::Mat m = cv::Mat::zeros(graph_slam_segmented.size(), CV_8U);
	gp.draw(m);
	cv::imshow("graph1 m", m);

	cv::Mat m2 = cv::Mat::zeros(graph_slam2_segmented.size(), CV_8U);
	gp2.draw(m2);
	cv::imshow("graph2 m", m);
	cv::waitKey(0);

	cv::imshow("TEST",graph_slam_segmented);
	draw(graph_slam, graph_slam2, graph_slam_segmented, graph_slam2_segmented, match);
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

//	double score = -1;
//	double old_score = -1;
//	int index_anchor = 0;

// 	auto vertex_anchor1 = match[0].source;
// 	auto vertex_anchor2 = match[0].target;

	std::deque < AASS::graphmatch::Match > anchors;
	std::deque < AASS::graphmatch::MatchLaplacian > anchors_laplacian;

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

		//Laplacian
		AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex_region_anchor_source;
		bool foundl = converter.getEquivalentLaplacianRegion(vertex_anchor1, vertex_region_anchor_source);
		AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex_region_anchor_target;
		bool found2l = converter2.getEquivalentLaplacianRegion(vertex_anchor2, vertex_region_anchor_target);

		assert(foundl == true);
		assert(found2l == true);

		std::cout << "Similarity of match added to anchors : " << match[i].getSimilarity() << std::endl;

		AASS::graphmatch::MatchLaplacian match_p_l(vertex_region_anchor_source, vertex_region_anchor_target);
		anchors_laplacian.push_back(match_p_l);




	}

	cv::waitKey(0);


	/************************************************************
	 * ANCHOR MATCHING
	 */

	for(auto anchor : anchors_laplacian) {
		gp_laplacian.addAnchor(anchor.getFirst());
		gp2_laplacian.addAnchor(anchor.getSecond());
	}


	/********** GRAPH LAPLACIAN ****************************/

	//Not using uniqueness score for now
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp3;
	for (vp3 = boost::vertices(gp_laplacian); vp3.first != vp3.second; ++vp3.first) {
		auto v = *vp3.first;
		gp_laplacian[v].setValue(1);
	}
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp2;
	for (vp2 = boost::vertices(gp2_laplacian); vp2.first != vp2.second; ++vp2.first) {
		auto v = *vp2.first;
		gp2_laplacian[v].setValue(1);
	}

	gp_laplacian.eigenLaplacian();
	gp2_laplacian.eigenLaplacian();

	/********** LAPLACIAN FAMILY SIGNATURES ****************/

	for(double time = 0 ; time < 10 ; time =  time + 0.5) {

		gp2_laplacian.propagateHeatKernel(time);
		gp_laplacian.propagateHeatKernel(time);

		/********** GRAPH MATCHING ****************************/

		auto hungarian_matches = gp_laplacian.hungarian_matching(gp2_laplacian);
		std::cout << "TIME " << time << std::endl;
		gp_laplacian.print();
		seeHungarianLaplacian(hungarian_matches, gp_laplacian, gp2_laplacian, graph_slam_segmented, graph_slam2_segmented);
	}



// 	while (old_score == -1 || score < old_score){
//	++index_anchor;
//	//MY THING
//	bool draw = true;
//	graphmatcheranchor.anchorMatching(gp, gp2, anchors, draw, slam1.size() );
//	std::deque<	AASS::graphmatch::Hypothese	> hypothesis_final = graphmatcheranchor.getResult();
//	graphmatcheranchor.sort(hypothesis_final);
//
//	old_score = score;
//	score = hypothesis_final[0].getDist();
//
//	hypothesis_final[0].drawHypo(gp, gp2, slam1, slam2, "finaleeee :D", 1);
//	cv::waitKey(0);
//
//	std::cout << "Score " << score << " old_score " << old_score << std::endl;

// 		vertex_anchor1 = match[index_anchor].source;
// 		vertex_anchor2 = match[index_anchor].target;

// 		converter.getEquivalentVPlace(vertex_anchor1, vertex_place_anchor_source);
// 		converter2.getEquivalentVPlace(vertex_anchor2, vertex_place_anchor_target);
// 		AASS::graphmatch::Match match_tmp(vertex_place_anchor_source, vertex_place_anchor_target);
// 		anchors.push_back(match_tmp);
// 	}

}