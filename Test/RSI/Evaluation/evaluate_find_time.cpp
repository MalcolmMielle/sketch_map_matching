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
#include <fstream>

#include "MapComparator/Hypothese.hpp"
#include "MapComparator/GraphMatcherNeighbor.hpp"
#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"

#include "Timed.hpp"
#include "vfl_utils_evaluation.hpp"



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


auto create_graphs_laplacian(const std::string& map_input, const std::string& map_model, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme){

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

	gp_laplacian->useHeatAnchors(use_anchor_heat);
	gp_laplacian_model->useHeatAnchors(use_anchor_heat);


	/********** GRAPH LAPLACIAN ****************************/

	//Not using uniqueness score for now

	if(use_relative_size_as_weight) {
		gp_laplacian->useRelativeSizeAsWeights();
		gp_laplacian_model->useRelativeSizeAsWeights();
	}
	else if(use_uniqueness_score){
		gp_laplacian->useUniquenessScoreAsWeights();
		gp_laplacian_model->useUniquenessScoreAsWeights();
	}
	else{
		gp_laplacian->noWeightForVertices();
		gp_laplacian_model->noWeightForVertices();
	}
	gp_laplacian->eigenLaplacian();
	gp_laplacian_model->eigenLaplacian();


	gp_laplacian->useOldComparisonMethod(use_old_matching_scheme);
	gp_laplacian_model->useOldComparisonMethod(use_old_matching_scheme);

	if(use_old_matching_scheme == true) {
		assert(gp_laplacian->isUsingOldMethod());
		assert(gp_laplacian_model->isUsingOldMethod());
	}

	return std::make_tuple(gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model);

}



auto match_maps_vfl(const std::string& map_input, const std::string& map_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {

	auto[gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(
			map_input, map_model, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme);

	if(use_old_matching_scheme == true) {
		assert(gp_laplacian->isUsingOldMethod());
		assert(gp_laplacian_model->isUsingOldMethod());
	}

	/********** LAPLACIAN FAMILY SIGNATURES ****************/

	int aninput = 0;
	double F1_good = -1;
	double precision = -1;
	double recall = -1;
	double good_time = -1;
	double tp_good = -1, fp_good = -1, fn_good = -1;
	for(double time = 0 ; time < 10 && aninput == 0; time =  time + 0.5) {

		gp_laplacian->propagateHeatKernel(time);
		gp_laplacian_model->propagateHeatKernel(time);
		/****************************************************************************
	 * Conversion
	 *
	 * **************************************************************************/
		std::map< AASS::graphmatch::Region*, AASS::graphmatch::GraphLaplacian::VertexLaplacian > equivalent_laplacian, equivalent_laplacian_model;

		ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graph_vfl = AASS::graphmatch::evaluation::graphLaplacian2VFL(*gp_laplacian, equivalent_laplacian);
		ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graph_vfl_model = AASS::graphmatch::evaluation::graphLaplacian2VFL(*gp_laplacian_model, equivalent_laplacian_model);

//		std::cout << "Size of input " << graph_slam.getNumVertices() << std::endl;
//		std::cout << "Size of model " << graph_slam_model.getNumVertices() << std::endl;

		/****************************************************************************
	 * Matching
	 *
	 * **************************************************************************/

		//Smallest graph needs to be first
		bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region*> > hyp_vfl;
		if(graph_vfl.NodeCount() < graph_vfl_model.NodeCount()){
			hyp_vfl = AASS::graphmatch::evaluation::matchTest(graph_vfl, graph_vfl_model);
		}
		else{
			hyp_vfl = AASS::graphmatch::evaluation::matchTest(graph_vfl_model, graph_vfl);
		}

//		AASS::graphmatch::HypotheseLaplacian hyp_res;
//

//		for(auto match : hyp_vfl.getAllElements()){
//			auto v1 = equivalent_laplacian[match.getFirst()];
//			auto v2 = equivalent_laplacian_model[match.getSecond()];
//
//			AASS::graphmatch::MatchLaplacian ma(v1, v2);
//			hyp_res.push_back(ma);
//		}
//
//		assert(hyp_res.size() == hyp_vfl.size());

		/********** GRAPH MATCHING ****************************/

		if(hyp_vfl.size() > 0) {
			// 					hypothesis_final_custom[0].drawMoved(gp_voro, gp_voro_model, drawing, drawing, "ALL FINAL CUSTOM Moved", 1);
			//		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1);



			//TODO SWicth grpah_slam_segmented to heat map

//			cv::Mat draw_tmp;
//			graph_slam_segmented.copyTo(draw_tmp);
//			gp_laplacian->drawSpecial(draw_tmp);
//
//			cv::Mat draw_tmp_model;
//			graph_slam_segmented_model.copyTo(draw_tmp_model);
//			gp_laplacian_model->drawSpecial(draw_tmp_model);
//	//
//			hypothesis_final_custom[0].drawLinks(*gp_laplacian, *gp_laplacian_model, draw_tmp, draw_tmp_model, "ALL FINAL CUSTOM ", 1);
//	//// 					std::string na = name + "_partial";
//	//// 					hypothesis_final_custom[0].drawPartialGraphs(gp_voro, gp_voro_model, input, test_model, na, 1, true);
//	//
//	//		std::cout << "Distance custom " << hypothesis_final_custom[0].getDist() << std::endl;
//	//
//	//		//EXPORT
//	////		cv::Mat drawing_out;
//	////		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1, drawing_out);
//	////		cv::imshow("Final", drawing_out);
//	//
//	////		cv::imwrite("RESULT.jpg", drawing_out);
//	//
//			std::cout << "Time : " << time << std::endl;
//			cv::waitKey(0);
			//
			//		std::cout << "Input 0 if not good and anything otherwise" << std::endl;
			//		std::cin >> aninput;


			std::cout << "Read file"<< gt_file  << std::endl;
			AASS::graphmatch::evaluation::Evaluation ev;
			ev.read_file(gt_file);
			std::cout << "Read file" << std::endl;

			auto [tp, fp, fn, prec, rec, F1] = ev.evaluate(hyp_vfl, *gp_laplacian, *gp_laplacian_model);

			if (F1_good == -1 || F1 > F1_good) {
				F1_good = F1;
				good_time = time;
				precision = prec;
				recall = rec;
				tp_good = tp;
				fp_good = fp;
				fn_good = fn;
			}

			if(all_results.find( time ) != all_results.end() ) {
				auto data = all_results.find(time)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(time)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(time);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(time, data));
			}
		}

		std::cout << "Running on :\n" << gt_file << " with time " << time << " res " << tp_good << " " << F1_good << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

	}


	delete gp_laplacian;
	delete gp_laplacian_model;

	if (F1_good == -1) {
		F1_good = 0;
		good_time = -1;
		precision = 0;
		recall = 0;
		tp_good = fp_good = fn_good = 0;
	}

	return std::make_tuple(tp_good, fp_good, fn_good, precision, recall, F1_good, good_time);



}


auto match_maps_hungarian(const std::string& map_input, const std::string& map_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {

	auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(map_input, map_model, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme);

	if(use_old_matching_scheme == true) {
		assert(gp_laplacian->isUsingOldMethod());
		assert(gp_laplacian_model->isUsingOldMethod());
	}

	/********** LAPLACIAN FAMILY SIGNATURES ****************/

	int aninput = 0;
	double F1_good = -1;
	double precision = -1;
	double recall = -1;
	double good_time = -1;
	double tp_good = -1, fp_good = -1, fn_good = -1;

//	std::make_tuple(tp_good, fp_good, fn_good, precision, recall, F1_good, good_time);

	for(double time = 0 ; time < 10 && aninput == 0; time =  time + 0.5) {

		gp_laplacian->propagateHeatKernel(time);
		gp_laplacian_model->propagateHeatKernel(time);

		/********** GRAPH MATCHING ****************************/


//		graphmatch_evg.planarEditDistanceAlgorithm(gp, gp_model);
		auto hungarian_matches = gp_laplacian->hungarian_matching(*gp_laplacian_model);
		std::cout << "DONE" << std::endl;

//		int rows = 0;
//		if(graph_slam_segmented.rows > graph_slam_segmented_model.rows){
//			rows = graph_slam_segmented.rows;
//		}
//		else{
//			rows = graph_slam_segmented_model.rows;
//		}
//		int cols = 0;
//		if(graph_slam_segmented.cols > graph_slam_segmented_model.cols){
//			cols = graph_slam_segmented.cols;
//		}
//		else{
//			cols = graph_slam_segmented_model.cols;
//		}

//		cv::Mat drawing = cv::Mat::zeros(rows , cols, CV_8UC3);

//		double perc = 0;
//		std::deque<
//				AASS::graphmatch::HypotheseLaplacian
//		> hypothesis_final_custom = graphmatch_custom.getResult();

		if(hungarian_matches.size() > 0) {
			// 					hypothesis_final_custom[0].drawMoved(gp_voro, gp_voro_model, drawing, drawing, "ALL FINAL CUSTOM Moved", 1);
			//		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1);



			//TODO SWicth grpah_slam_segmented to heat map

//			cv::Mat draw_tmp;
//			graph_slam_segmented.copyTo(draw_tmp);
//			gp_laplacian->drawSpecial(draw_tmp);
//
//			cv::Mat draw_tmp_model;
//			graph_slam_segmented_model.copyTo(draw_tmp_model);
//			gp_laplacian_model->drawSpecial(draw_tmp_model);
//	//
//			hypothesis_final_custom[0].drawLinks(*gp_laplacian, *gp_laplacian_model, draw_tmp, draw_tmp_model, "ALL FINAL CUSTOM ", 1);
//	//// 					std::string na = name + "_partial";
//	//// 					hypothesis_final_custom[0].drawPartialGraphs(gp_voro, gp_voro_model, input, test_model, na, 1, true);
//	//
//	//		std::cout << "Distance custom " << hypothesis_final_custom[0].getDist() << std::endl;
//	//
//	//		//EXPORT
//	////		cv::Mat drawing_out;
//	////		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1, drawing_out);
//	////		cv::imshow("Final", drawing_out);
//	//
//	////		cv::imwrite("RESULT.jpg", drawing_out);
//	//
//			std::cout << "Time : " << time << std::endl;
//			cv::waitKey(0);
			//
			//		std::cout << "Input 0 if not good and anything otherwise" << std::endl;
			//		std::cin >> aninput;


			std::cout << "Read file" << std::endl;
			AASS::graphmatch::evaluation::Evaluation ev;
			ev.read_file(gt_file);
			std::cout << "Read file" << std::endl;

			auto [tp, fp, fn, prec, rec, F1] = ev.evaluate(hungarian_matches, *gp_laplacian, *gp_laplacian_model);

			if (F1_good == -1 || F1 > F1_good) {
				F1_good = F1;
				good_time = time;
				precision = prec;
				recall = rec;
				tp_good = tp;
				fp_good = fp;
				fn_good = fn;
			}
			if(all_results.find( time ) != all_results.end() ) {
				auto data = all_results.find(time)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(time)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(time);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(time, data));
			}
		}
//		else{
//
//			all_results.push_back( std::make_tuple(tp, fp, fn, prec, rec, F1) );
//		}


		std::cout << "Running on :\n" << gt_file << " with time " << time << " res " << tp_good << " " << F1_good << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

	}


	delete gp_laplacian;
	delete gp_laplacian_model;

//	if (F1_good == -1) {
//		F1_good = 0;
//		good_time = -1;
//	}
	if (F1_good == -1) {
		F1_good = 0;
		good_time = -1;
		precision = 0;
		recall = 0;
		tp_good = fp_good = fn_good = 0;
	}

	return std::make_tuple(tp_good, fp_good, fn_good, precision, recall, F1_good, good_time);

}



auto match_maps_and_find_time(const std::string& map_input, const std::string& map_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {


	auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(map_input, map_model, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme);

	if(use_old_matching_scheme == true) {
		assert(gp_laplacian->isUsingOldMethod());
		assert(gp_laplacian_model->isUsingOldMethod());
	}

	/********** LAPLACIAN FAMILY SIGNATURES ****************/

	int aninput = 0;
	double F1_good = -1;
	double precision = -1;
	double recall = -1;
	double good_time = -1;
	double tp_good = -1, fp_good = -1, fn_good = -1;
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
		std::cout << "DONE" << std::endl;

//		int rows = 0;
//		if(graph_slam_segmented.rows > graph_slam_segmented_model.rows){
//			rows = graph_slam_segmented.rows;
//		}
//		else{
//			rows = graph_slam_segmented_model.rows;
//		}
//		int cols = 0;
//		if(graph_slam_segmented.cols > graph_slam_segmented_model.cols){
//			cols = graph_slam_segmented.cols;
//		}
//		else{
//			cols = graph_slam_segmented_model.cols;
//		}

//		cv::Mat drawing = cv::Mat::zeros(rows , cols, CV_8UC3);

//		double perc = 0;
		std::deque<
				AASS::graphmatch::HypotheseLaplacian
		> hypothesis_final_custom = graphmatch_custom.getResult();

		if(hypothesis_final_custom.size() > 0) {
			std::cout << "Sorting" << std::endl;

			graphmatch_custom.sort(hypothesis_final_custom);
			std::cout << "Sorted" << std::endl;
	// 					hypothesis_final_custom[0].drawMoved(gp_voro, gp_voro_model, drawing, drawing, "ALL FINAL CUSTOM Moved", 1);
	//		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1);



			//TODO SWicth grpah_slam_segmented to heat map

//			cv::Mat draw_tmp;
//			graph_slam_segmented.copyTo(draw_tmp);
//			gp_laplacian->drawSpecial(draw_tmp);
//
//			cv::Mat draw_tmp_model;
//			graph_slam_segmented_model.copyTo(draw_tmp_model);
//			gp_laplacian_model->drawSpecial(draw_tmp_model);
//	//
//			hypothesis_final_custom[0].drawLinks(*gp_laplacian, *gp_laplacian_model, draw_tmp, draw_tmp_model, "ALL FINAL CUSTOM ", 1);
//	//// 					std::string na = name + "_partial";
//	//// 					hypothesis_final_custom[0].drawPartialGraphs(gp_voro, gp_voro_model, input, test_model, na, 1, true);
//	//
//	//		std::cout << "Distance custom " << hypothesis_final_custom[0].getDist() << std::endl;
//	//
//	//		//EXPORT
//	////		cv::Mat drawing_out;
//	////		hypothesis_final_custom[0].drawHypo(*gp_laplacian, *gp_laplacian_model, drawing, drawing, "ALL FINAL CUSTOM", 1, drawing_out);
//	////		cv::imshow("Final", drawing_out);
//	//
//	////		cv::imwrite("RESULT.jpg", drawing_out);
//	//
//			std::cout << "Time : " << time << std::endl;
//			cv::waitKey(0);
	//
	//		std::cout << "Input 0 if not good and anything otherwise" << std::endl;
	//		std::cin >> aninput;


			std::cout << "Read file " << gt_file << std::endl;
			AASS::graphmatch::evaluation::Evaluation ev;
			ev.read_file(gt_file);
			std::cout << "Read file" << std::endl;

			auto [tp, fp, fn, prec, rec, F1] = ev.evaluate(hypothesis_final_custom[0], *gp_laplacian, *gp_laplacian_model);

			if (F1_good == -1 || F1 > F1_good) {
				F1_good = F1;
				good_time = time;
				precision = prec;
				recall = rec;
				tp_good = tp;
				fp_good = fp;
				fn_good = fn;
			}

			if(all_results.find( time ) != all_results.end() ) {
				auto data = all_results.find(time)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(time)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(time);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(time, data));
			}
		}

		std::cout << "Running on :\n" << gt_file << " with time " << time << " res " << tp_good << " " << F1_good << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

	}


	delete gp_laplacian;
	delete gp_laplacian_model;

	if (F1_good == -1) {
		F1_good = 0;
		good_time = -1;
		precision = 0;
		recall = 0;
		tp_good = fp_good = fn_good = 0;
	}

	return std::make_tuple(tp_good, fp_good, fn_good, precision, recall, F1_good, good_time);

}


//auto fuse_detailed_results(const std::map< double, std::tuple<double, double, double, double, double, double> >& results, std::map< double, std::tuple<double, double, double, double, double, double> >& all_results_out){
//
//	for (auto element : results){
//
//		//Merge results
//		if(all_results_out.find( element.first ) != all_results_out.end() ){
//
//			auto [tp, fp, fn, prec, rec, F1] = all_results_out[element.first];
//			auto [tp_r, fp_r, fn_r, prec_r, rec_r, F1_r] = element.second;
//			all_results_out[element.first] = std::make_tuple(tp + tp_r, fp + fp_r, fn + fn_r , prec + prec_r, rec + rec_r, F1 + F1_r);
//
//		}
//		else{
//			all_results_out[element.first] = element.second;
//		}
//	}
//}


auto export_mean_std_detailed_results_mean(const std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results_out, const std::string& file_out){

	std::string result_file = file_out;
	std::ofstream myfile;
	if (!AASS::graphmatch::evaluation::exists_test3(result_file)) {
		myfile.open(result_file);
		myfile << "# time meantp std meanfp std meanfn std meanprecision std meanrecall std meanF1 std\n";
	} else {
		myfile.open(result_file, std::ios::out | std::ios::app);
//		myfile << "# time meantp std meanfp std meanfn std meanprecision std meanrecall std meanF1 std\n";
	}

	if (myfile.is_open()) {
		for(auto element : all_results_out) {
//			myfile << "# element " << element.first << "\n";
			element.second.export_mean_data(myfile);
		}
	}


}


auto export_mean_std_detailed_results(const std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results_out, const std::string& file_out){

	std::string result_file = file_out;
	std::ofstream myfile;
	if (!AASS::graphmatch::evaluation::exists_test3(result_file)) {
		myfile.open(result_file);
		myfile << "# map tp fp fn precision recall F1 time\n";
	} else {
		myfile.open(result_file, std::ios::out | std::ios::app);
		myfile << "# map tp fp fn precision recall F1 time\n";
	}

	if (myfile.is_open()) {
		for(auto element : all_results_out) {
			if( element.first == static_cast<int>(element.first)){
				myfile << "# element " << element.first << "\n";
				element.second.export_detailed_data(myfile);
			}
		}
		myfile << "\n\n\n";
		for(auto element : all_results_out) {
			if( element.first != static_cast<int>(element.first)){
				myfile << "# element " << element.first << "\n";
				element.second.export_detailed_data(myfile);
			}
		}
	}


}


auto evaluate_all_files(const std::string& input_folder, const std::string& gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, const std::string& prefix_details){

	std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results_detailed;


	auto rec = std::experimental::filesystem::directory_iterator(input_folder);
	int count = 0;
	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {

		auto p_canon = std::experimental::filesystem::canonical(*p);
		if(!std::experimental::filesystem::is_directory(p_canon) ){
			auto input_file_stem =p_canon.stem();

			if(input_file_stem.string().compare("model_simple") != 0) {
				count++;

				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
				std::string gt_file = gt_folder + "/" + gt_name;

				std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png"
				          << std::endl;

//				std::map< double, std::tuple<double, double, double, double, double, double> > all_results_tmp;
				auto[tp, fp, fn, prec, rec, F1, time] = match_maps_and_find_time(p_canon.string(), input_folder + "/model_simple.png",
				                                            gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, all_results_detailed);

				results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));
//				fuse_detailed_results(all_results_tmp, all_results_detailed);
			}

		}

	}
	export_mean_std_detailed_results(all_results_detailed, "export_detailed_" + prefix_details +".dat");
	export_mean_std_detailed_results_mean(all_results_detailed, "export_detailed_" + prefix_details +"_all_summarized.dat");
	return results;


}

auto evaluate_all_files_hungarian(const std::string& input_folder, const std::string& gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, const std::string& prefix_details){

	std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results_detailed;

	auto rec = std::experimental::filesystem::directory_iterator(input_folder);

	int count = 0;


	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {

		auto p_canon = std::experimental::filesystem::canonical(*p);
		if(!std::experimental::filesystem::is_directory(p_canon) ){
			auto input_file_stem =p_canon.stem();

			if(input_file_stem.string().compare("model_simple") != 0) {

				count++;

				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
				std::string gt_file = gt_folder + "/" + gt_name;

				std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png"
				          << std::endl;

//				std::map< double, std::tuple<double, double, double, double, double, double> > all_results_tmp;
				auto[tp, fp, fn, prec, rec, F1, time] = match_maps_hungarian(p_canon.string(), input_folder + "/model_simple.png",
				                                                             gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, all_results_detailed);

				results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));

//				fuse_detailed_results(all_results_tmp, all_results_detailed);

			}

		}

	}

	export_mean_std_detailed_results(all_results_detailed, "export_detailed_hungarian_" + prefix_details +".dat");
	export_mean_std_detailed_results_mean(all_results_detailed, "export_detailed_hungarian_" + prefix_details +"_all_summarized.dat");

	return results;


}


auto evaluate_all_files_vfl(const std::string& input_folder, const std::string& gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, const std::string& prefix_details){

	std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results_detailed;
	int count = 0;


	auto rec = std::experimental::filesystem::directory_iterator(input_folder);
	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {

		auto p_canon = std::experimental::filesystem::canonical(*p);
		if(!std::experimental::filesystem::is_directory(p_canon) ){
			auto input_file_stem =p_canon.stem();

			if(input_file_stem.string().compare("model_simple") != 0) {
				count++;

				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
				std::string gt_file = gt_folder + "/" + gt_name;

				std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png"
				          << std::endl;

//				std::map< double, std::tuple<double, double, double, double, double, double> > all_results_tmp;
				auto[tp, fp, fn, prec, rec, F1, time] = match_maps_vfl(p_canon.string(), input_folder + "/model_simple.png",
				                                                             gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, all_results_detailed);

				results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));

//				fuse_detailed_results(all_results_tmp, all_results_detailed);
			}

		}

	}
	export_mean_std_detailed_results(all_results_detailed, "export_detailed_vfl_" + prefix_details +".dat");
	export_mean_std_detailed_results_mean(all_results_detailed, "export_detailed_vfl_" + prefix_details +"_all_summarized.dat");
	return results;


}


void print_results(const std::vector<std::tuple<std::string, double, double, double, double, double, double, double > >& results){
	double sum = 0;

	for (auto result : results){
		std::cout << std::get<0>(result) << " -> tp, " << std::get<1>(result) << " fp, " << std::get<2>(result) << " fn " << std::get<3>(result) << " precision " << std::get<4>(result) << " recall " << std::get<5>(result) << " F1 " << std::get<6>(result) << " at time " << std::get<7>(result) << std::endl;
		sum +=  std::get<6>(result);
	}

	std::cout << "Final result : " << sum / results.size() << std::endl;
	std::cout << "END" << std::endl;
}


void export_results(const std::string& file_out, const std::vector<std::tuple<std::string, double, double, double, double, double, double, double > >& results){

	double sum = 0;
	double sum_precision = 0;
	double sum_recall = 0;

	std::string result_file = file_out;
	std::ofstream myfile;
	if (!AASS::graphmatch::evaluation::exists_test3(result_file)) {
		myfile.open(result_file);
		myfile << "# map tp fp fn precision recall F1 time\n";
	} else {
		myfile.open(result_file, std::ios::out | std::ios::app);
		myfile << "# map tp fp fn precision recall F1 time\n";
	}

	if (myfile.is_open()) {
		for (auto result : results){
			myfile << std::get<0>(result) << " " << std::get<1>(result) << " " << std::get<2>(result) << " " << std::get<3>(result) << " " << std::get<4>(result) << " " << std::get<5>(result) << " " << std::get<6>(result) << " " << std::get<7>(result);
			myfile << "\n";
			sum +=  std::get<6>(result);
			sum_precision +=  std::get<4>(result);
			sum_recall +=  std::get<5>(result);
		}
	}

	double std_sum = 0;
	double std_mean_precision = 0;
	double std_mean_recall = 0;
	double mean = sum / results.size();
	double mean_precision = sum_precision / results.size();
	double mean_recall = sum_recall / results.size();
	for (auto result : results) {
		std_sum += (std::get<6>(result) - mean) * (std::get<6>(result) - mean);
		std_mean_precision += (std::get<4>(result) - mean_precision) * (std::get<4>(result) - mean_precision);
		std_mean_recall += (std::get<5>(result) - mean_recall) * (std::get<5>(result) - mean_recall);
	}

	std_sum = std::sqrt(std_sum / (results.size() - 1 ) );
	std_mean_precision = std::sqrt(std_mean_precision / (results.size() - 1 ) );
	std_mean_recall = std::sqrt(std_mean_recall / (results.size() - 1 ) );


	myfile << "\n\n# F1mean std precisionmean std recallmean std \n";
	myfile << mean << " " << std_sum << " " << mean_precision << " " << std_mean_precision << " " << mean_recall << " " << std_mean_recall << "\n";

}




int main(int argc, char** argv){


	std::string input_folder = "../../../../Test/RSI/Sketches";
	std::string gt_folder = "../../../../Test/RSI/Sketches/GT";



	auto results_base_old_method =  evaluate_all_files(input_folder, gt_folder, false, false, false, true, "base_old");
	std::cout << "Results base_old_method" << std::endl;
	print_results(results_base_old_method);
	export_results("results_base_old_method.dat", results_base_old_method);

	auto results_anchors_old_method =  evaluate_all_files(input_folder, gt_folder, true, false, false, true, "anchors_old");
	std::cout << "Results Anchors_old_method" << std::endl;
	print_results(results_anchors_old_method);
	export_results("results_anchors_old_method.dat", results_anchors_old_method);

	auto results_anchors_uniqueness_old_method =  evaluate_all_files(input_folder, gt_folder, true, true, false, true, "anchors_unique_old");
	auto results_anchors_relative_size_old_method =  evaluate_all_files(input_folder, gt_folder, true, false, true, true, "anchors_relative_size_old");

	std::cout << "Results Anchors Uniqueness_old_method" << std::endl;
	print_results(results_anchors_uniqueness_old_method);
	export_results("results_anchors_uniqueness_old_method.dat", results_anchors_uniqueness_old_method);
	std::cout << "Results Anchors Relative size_old_method" << std::endl;
	print_results(results_anchors_relative_size_old_method);
	export_results("results_anchors_relative_size_old_method.dat", results_anchors_relative_size_old_method);


	auto results_base =  evaluate_all_files(input_folder, gt_folder, false, false, false, false, "base");
	std::cout << "Results base" << std::endl;
	print_results(results_base);
	export_results("results_base.dat", results_base);

	auto results_anchors =  evaluate_all_files(input_folder, gt_folder, true, false, false, false, "anchors");
	auto results_anchors_uniqueness =  evaluate_all_files(input_folder, gt_folder, true, true, false, false, "anchors_unique");
	auto results_anchors_relative_size =  evaluate_all_files(input_folder, gt_folder, true, false, true, false, "anchors_relative_size");

	std::cout << "Results Anchors" << std::endl;
	print_results(results_anchors);
	export_results("results_anchors.dat", results_anchors);
	std::cout << "Results Anchors Uniqueness" << std::endl;
	print_results(results_anchors_uniqueness);
	export_results("results_anchors_uniqueness.dat", results_anchors_uniqueness);
	std::cout << "Results Anchors Relative size" << std::endl;
	print_results(results_anchors_relative_size);
	export_results("results_anchors_relative_size.dat", results_anchors_relative_size);




	auto results_base_hungarian =  evaluate_all_files_hungarian(input_folder, gt_folder, false, false, false, false, "base");
	auto results_anchors_hungarian =  evaluate_all_files_hungarian(input_folder, gt_folder, true, false, false, false, "anchors");
	auto results_anchors_uniqueness_hungarian =  evaluate_all_files_hungarian(input_folder, gt_folder, true, true, false, false, "anchors_unique");
	auto results_anchors_relative_size_hungarian =  evaluate_all_files_hungarian(input_folder, gt_folder, true, false, true, false, "anchors_relative_size");
	auto results_old_method_hungarian =  evaluate_all_files_hungarian(input_folder, gt_folder, true, false, true, true, "old_method");

	std::cout << "Results base_hungarian" << std::endl;
	print_results(results_base_hungarian);
	export_results("results_base_hungarian.dat", results_base_hungarian);
	std::cout << "Results Anchors_hungarian" << std::endl;
	print_results(results_anchors_hungarian);
	export_results("results_anchors_hungarian.dat", results_anchors_hungarian);
	std::cout << "Results Anchors Uniqueness_hungarian" << std::endl;
	print_results(results_anchors_uniqueness_hungarian);
	export_results("results_anchors_uniqueness_hungarian.dat", results_anchors_uniqueness_hungarian);
	std::cout << "Results Anchors Relative size_hungarian" << std::endl;
	print_results(results_anchors_relative_size_hungarian);
	export_results("results_anchors_relative_size_hungarian.dat", results_anchors_relative_size_hungarian);
	std::cout << "Results Old Method hungarian" << std::endl;
	print_results(results_old_method_hungarian);
	export_results("results_old_method_hungarian.dat", results_old_method_hungarian);

	auto results_base_vfl =  evaluate_all_files_vfl(input_folder, gt_folder, false, false, false, false, "base");
	auto results_anchors_vfl =  evaluate_all_files_vfl(input_folder, gt_folder, true, false, false, false, "anchors");
	auto results_anchors_uniqueness_vfl =  evaluate_all_files_vfl(input_folder, gt_folder, true, true, false, false, "anchors_unique");
	auto results_anchors_relative_size_vfl =  evaluate_all_files_vfl(input_folder, gt_folder, true, false, true, false, "anchors_relative_size");
	auto results_old_method_vfl =  evaluate_all_files_vfl(input_folder, gt_folder, true, false, true, true, "old_method");


	std::cout << "Results base_vfl" << std::endl;
	print_results(results_base_vfl);
	export_results("results_base_vfl.dat", results_base_vfl);
	std::cout << "Results Anchors_vfl" << std::endl;
	print_results(results_anchors_vfl);
	export_results("results_anchors_vfl.dat", results_anchors_vfl);
	std::cout << "Results Anchors Uniqueness_vfl" << std::endl;
	print_results(results_anchors_uniqueness_vfl);
	export_results("results_anchors_uniqueness_vfl.dat", results_anchors_uniqueness_vfl);
	std::cout << "Results Anchors Relative size_vfl" << std::endl;
	print_results(results_anchors_relative_size_vfl);
	export_results("results_anchors_relative_size_vfl.dat", results_anchors_relative_size_vfl);
	std::cout << "Results Old Method vfl" << std::endl;
	print_results(results_old_method_vfl);
	export_results("results_old_method_vfl.dat", results_old_method_vfl);


	//HACK because can't copy iterator
	//	int count = 0;



}