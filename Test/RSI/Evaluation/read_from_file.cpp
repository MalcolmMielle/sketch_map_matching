#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
//#include <RSI/FuzzyOpening.hpp>
//#include <RSI/ZoneReducer.hpp>
//#include <RSI/ZoneExtractor.hpp>
#include <RSI/Uniqueness.hpp>

#include "RSI/GraphZoneRI.hpp"

#include "maoris/ZoneExtractor.hpp"
#include "maoris/FuzzyOpening.hpp"
//#include "maoris/Kmean.hpp"
#include "maoris/ZoneReducer.hpp"
#include "maoris/Segmentor.hpp"

#include "MatchMaps.hpp"
#include "RSIConversion.hpp"


#include "Evaluation.hpp"


AASS::graphmatch::GraphLaplacian::VertexLaplacian getBiggestRegion(const AASS::graphmatch::GraphLaplacian& gl){

//	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
	//vertices access all the vertix
	AASS::graphmatch::GraphLaplacian::VertexLaplacian out;
	double count = -1;
	for (auto vp = boost::vertices(gl.getGraph() ); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
//		gl[v].print();
		if(count < gl[v].zone.size() ){
			count = gl[v].zone.size();
			out = v;
		}
	}
	return out;

}

AASS::graphmatch::GraphLaplacian::VertexLaplacian getSmallestRegion(const AASS::graphmatch::GraphLaplacian& gl){

//	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
	//vertices access all the vertix
	AASS::graphmatch::GraphLaplacian::VertexLaplacian out;
	double count = -1;
	for (auto vp = boost::vertices(gl.getGraph() ); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
//		gl[v].print();
		if(count > gl[v].zone.size() || count == -1){
			count = gl[v].zone.size();
			out = v;
		}
	}
	return out;

}


cv::Mat makeGraph(const std::string& file, AASS::RSI::GraphZoneRI& graph_slam){

	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
/** Segmenting the map**/
	AASS::maoris::Segmentor segmenteur;
	AASS::maoris::GraphZone graph_segmented;

	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	time = segmenteur.segmentImage(slam1, graph_segmented);
	cv::Mat segmented_map = segmenteur.getSegmentedMap();

//	cv::imshow("Segmented", segmented_map);
//	cv::waitKey(0);

	graph_slam = AASS::RSI::GraphZoneRI(graph_segmented);

	graph_slam.updatePCA();
	graph_slam.setPCAClassification();
	graph_slam.setSizesClassification();

	return segmented_map;
}


BOOST_AUTO_TEST_CASE(trying){

	std::string file_export = "export.dat";


	AASS::graphmatch::evaluation::Evaluation ev;
	ev.read_file(file_export);


	std::cout << "Creating graphs" << std::endl;

	std::string file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/01.png";
	AASS::RSI::GraphZoneRI graph_slam_model;
	cv::Mat graph_slam_segmented = makeGraph(file, graph_slam_model);


	std::string file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/02.png";
	AASS::RSI::GraphZoneRI graph_slam_model2;
	cv::Mat graph_slam_segmented2 = makeGraph(file2, graph_slam_model2);

	graph_slam_model.setSDAwayFromMeanForUniqueness(1);
	graph_slam_model2.setSDAwayFromMeanForUniqueness(1);

	/********** Uniqueness *******************************************/
	graph_slam_model.updateUnique();
	graph_slam_model2.updateUnique(graph_slam_model);


	AASS::graphmatch::GraphLaplacian gp_laplacian;
	AASS::graphmatch::RSIGraphConverter converter;
//	converter.graphZoneToGraphPlace(graph_slam_model, gp);
	converter.graphZonetoGraphLaplacian(graph_slam_model, gp_laplacian);

	AASS::graphmatch::GraphLaplacian gp_laplacian_model;
	AASS::graphmatch::RSIGraphConverter converter2;
//	converter.graphZoneToGraphPlace(graph_slam_model, gp);
	converter2.graphZonetoGraphLaplacian(graph_slam_model2, gp_laplacian_model);

	auto b_1 = getBiggestRegion(gp_laplacian);
	auto b_2 = getBiggestRegion(gp_laplacian_model);

	AASS::graphmatch::MatchLaplacian match(b_1, b_2);

//	assert(ev.is_correct(match, gp_laplacian, gp_laplacian_model) == true);
// 	BOOST_CHECK(ev.is_correct(match, gp_laplacian, gp_laplacian_model));

	//Save centers in file


	auto s_1 = getSmallestRegion(gp_laplacian);
	auto s_2 = getSmallestRegion(gp_laplacian_model);

	AASS::graphmatch::MatchLaplacian match_small(s_1, s_2);
//	assert(ev.is_correct(match_small, gp_laplacian, gp_laplacian_model) == false);
// 	BOOST_CHECK(!ev.is_correct(match_small, gp_laplacian, gp_laplacian_model));

	AASS::graphmatch::HypotheseLaplacian hyp;
	hyp.push_back(match);
	auto[tp, fp, fn, prec, rec, F1] = ev.evaluate(hyp, gp_laplacian, gp_laplacian_model);
	BOOST_CHECK_EQUAL(F1, 1);

//	assert(perc == 1);

	hyp.push_back(match_small);
	auto[tp2, fp2, fn2, prec2, rec2, F12] = ev.evaluate(hyp, gp_laplacian, gp_laplacian_model);
	BOOST_CHECK_EQUAL(F12, 0.5);


	std::cout << "END" << std::endl;

}
