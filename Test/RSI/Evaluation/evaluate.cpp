#include <iostream>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <experimental/filesystem>
//#include <RSI/FuzzyOpening.hpp>
//#include <RSI/ZoneReducer.hpp>
//#include <RSI/ZoneExtractor.hpp>
#include "RSI/GraphZoneRI.hpp"
#include "RSI/ZoneCompared.hpp"
#include "RSI/hungarian/hungarian.h"
#include "RSI/HungarianMatcher.hpp"

#include "maoris/ZoneExtractor.hpp"
#include "maoris/FuzzyOpening.hpp"
//#include "maoris/Kmean.hpp"
#include "maoris/ZoneReducer.hpp"
#include "maoris/Segmentor.hpp"

#include "MatchMaps.hpp"
#include "RSIConversion.hpp"


#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"
#include "LaplacianGraphMatching/GraphMatcherNeighborLaplacian.hpp"


#include "Evaluation.hpp"


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



auto match_maps(const std::string& map_input, const std::string& map_model) {


	AASS::RSI::GraphZoneRI graph_slam;
	cv::Mat graph_slam_segmented = makeGraph(map_input, graph_slam);
	AASS::RSI::GraphZoneRI graph_slam_model;
	cv::Mat graph_slam_segmented_model = makeGraph(map_model, graph_slam_model);

	graph_slam.setSDAwayFromMeanForUniqueness(1);
	graph_slam_model.setSDAwayFromMeanForUniqueness(1);

	/********** Uniqueness *******************************************/
	graph_slam.updateUnique();
	graph_slam_model.updateUnique();

	/********** Hungarian matching of graph onto itself***************/

	std::cout << "Hungarian Match" << std::endl;
	AASS::RSI::HungarianMatcher hungmatch;
	std::vector<int> scores;
	auto match = hungmatch.match(graph_slam, graph_slam_model, scores);

// 	exit(0);

	std::sort(match.begin(), match.end(), [&graph_slam, &graph_slam_model](AASS::RSI::ZoneCompared &match, AASS::RSI::ZoneCompared &match1){
		return match.getSimilarity() < match1.getSimilarity();
	} );



	//SHOULD WORK :(. Sadly still copies https://stackoverflow.com/questions/51521031/return-stdtuple-and-move-semantics-copy-elision
//	std::tuple<AASS::graphmatch::HypotheseLaplacian, AASS::graphmatch::GraphLaplacian, AASS::graphmatch::GraphLaplacian> t_out;
//	auto& [hyp_out, gp_laplacian, gp_laplacian_model] = t_out;

	AASS::graphmatch::GraphLaplacian* gp_laplacian = new AASS::graphmatch::GraphLaplacian();
	AASS::graphmatch::RSIGraphConverter converter;
//	converter.graphZoneToGraphPlace(graph_slam_model, gp);
	converter.graphZonetoGraphLaplacian(graph_slam, *gp_laplacian);

	AASS::graphmatch::GraphLaplacian* gp_laplacian_model = new AASS::graphmatch::GraphLaplacian();
	AASS::graphmatch::RSIGraphConverter converter2;
//	converter.graphZoneToGraphPlace(graph_slam_model, gp);
	converter2.graphZonetoGraphLaplacian(graph_slam_model, *gp_laplacian_model);


	std::deque < AASS::graphmatch::Match > anchors;
	std::deque < AASS::graphmatch::MatchLaplacian > anchors_laplacian;

	for(size_t i = 0 ; i < match.size() ; ++i){

		auto vertex_anchor1 = match[i].source;
		auto vertex_anchor2 = match[i].target;
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

	/************************************************************
	 * ANCHOR MATCHING
	 */

	for(auto anchor : anchors_laplacian) {
		gp_laplacian->addAnchor(anchor.getFirst());
		gp_laplacian_model->addAnchor(anchor.getSecond());
	}


	/********** GRAPH LAPLACIAN ****************************/

	//Not using uniqueness score for now
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp3;
	for (vp3 = boost::vertices(*gp_laplacian_model); vp3.first != vp3.second; ++vp3.first) {
		auto v = *vp3.first;
		(*gp_laplacian_model)[v].setUniqueness(1);
	}
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp2;
	for (vp2 = boost::vertices(*gp_laplacian_model); vp2.first != vp2.second; ++vp2.first) {
		auto v = *vp2.first;
		(*gp_laplacian_model)[v].setUniqueness(1);
	}

	gp_laplacian->eigenLaplacian();
	gp_laplacian_model->eigenLaplacian();

	/********** LAPLACIAN FAMILY SIGNATURES ****************/

	int aninput = 0;
	for(double time = 0 ; time < 10 && aninput == 0; time =  time + 0.5) {

		gp_laplacian->propagateHeatKernel(time);
		gp_laplacian_model->propagateHeatKernel(time);

		/********** GRAPH MATCHING ****************************/

//		auto hungarian_matches = gp_laplacian.hungarian_matching(gp2_laplacian);
//		std::cout << "TIME " << time << std::endl;
//		gp_laplacian.print();
//		seeHungarianLaplacian(hungarian_matches, gp_laplacian, gp2_laplacian, g**raph_slam_segmented, graph_slam2_segmented);


//		AASS::graphmatch::GraphMatcherNeighborLaplacian graphmatch_evg;
		AASS::graphmatch::GraphMatcherNeighborLaplacian graphmatch_custom;
		// 	AASS::graphmatch::GraphMatcherClusterFiltered graphmatchold;

		//MY THING
//		graphmatch_evg.planarEditDistanceAlgorithm(gp, gp_model);
		graphmatch_custom.planarEditDistanceAlgorithm(*gp_laplacian, *gp_laplacian_model);

		int rows = 0;
		if(graph_slam_segmented.rows > graph_slam_segmented_model.rows){
			rows = graph_slam_segmented.rows;
		}
		else{
			rows = graph_slam_segmented_model.rows;
		}
		int cols = 0;
		if(graph_slam_segmented.cols > graph_slam_segmented_model.cols){
			cols = graph_slam_segmented.cols;
		}
		else{
			cols = graph_slam_segmented_model.cols;
		}

		cv::Mat drawing = cv::Mat::zeros(rows , cols, CV_8UC3);

		std::deque<
				AASS::graphmatch::HypotheseLaplacian
		> hypothesis_final_custom = graphmatch_custom.getResult();
		graphmatch_custom.sort(hypothesis_final_custom);
// 					hypothesis_final_custom[0].drawMoved(gp_voro, gp_voro_model, drawing, drawing, "ALL FINAL CUSTOM Moved", 1);
		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1);
		hypothesis_final_custom[0].drawLinks(*gp_laplacian, *gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model, "ALL FINAL CUSTOM ", 1);
// 					std::string na = name + "_partial";
// 					hypothesis_final_custom[0].drawPartialGraphs(gp_voro, gp_voro_model, input, test_model, na, 1, true);

		std::cout << "Distance custom " << hypothesis_final_custom[0].getDist() << std::endl;

		//EXPORT
		cv::Mat drawing_out;
		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1, drawing_out);

//		cv::imwrite("RESULT.jpg", drawing_out);

		cv::waitKey(0);

		std::cout << "Input 0 if not good and anything otherwise" << std::endl;
		std::cin >> aninput;

		if(aninput != 0){
//			hyp_out = hypothesis_final_custom[0];
//			return t_out;

			return std::make_tuple(hypothesis_final_custom[0], gp_laplacian, gp_laplacian_model);
		}

	}

	return std::make_tuple(AASS::graphmatch::HypotheseLaplacian(), gp_laplacian, gp_laplacian_model);



}



int main(){


	std::string input_folder = "../../../../Test/RSI/Sketches_small_test";
	std::string gt_folder = "../../../../Test/RSI/Sketches_small_test/GT";

	//HACK because can't copy iterator
	//	int count = 0;

	auto rec = std::experimental::filesystem::recursive_directory_iterator(input_folder);
	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {

		auto p_canon = std::experimental::filesystem::canonical(*p);
		if(!std::experimental::filesystem::is_directory(p_canon) ){
			auto input_file_stem =p_canon.stem();
			std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";

			std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png" << std::endl;

			auto [hyp, gp_laplacian, gp_laplacian_model] = match_maps(p_canon.string(), input_folder + "/model_simple.png");

			AASS::graphmatch::evaluation::Evaluation ev;
			ev.read_file(gt_folder + "/" + gt_name);

			std::cout << "Running on :\n" << gt_folder + "/" + gt_name << " And the graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

			double perc = ev.evaluate(hyp, *gp_laplacian, *gp_laplacian_model);
			std::cout << "\nRESULT:\n" << perc << "%\n" << std::endl;

			delete gp_laplacian;
			delete gp_laplacian_model;

			exit(0);

		}

	}


	std::cout << "END" << std::endl;

}