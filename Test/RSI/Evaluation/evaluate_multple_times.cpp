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

#include "CreateHypFromGT.hpp"



bool is_sketch = false;


cv::Mat makeGraph(const std::string& file, AASS::RSI::GraphZoneRI& graph_slam){

	std::cout << "Loading " << file << std::endl;
	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
/** Segmenting the map**/
	AASS::maoris::Segmentor segmenteur;
	AASS::maoris::GraphZone graph_segmented;


	if(is_sketch) {
		graph_segmented.setThreshold(0.30);
	}
	else{
		graph_segmented.setThreshold(0.50);
	}
	graph_segmented.setMargin(0.1);
	graph_segmented.setThresholdFusionRipples(40);
	if(is_sketch) {
		graph_segmented.setThresholdFusionDoors(60);
	}
	else{
		graph_segmented.setThresholdFusionDoors(40);
	}

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


auto create_graphs_laplacian(AASS::RSI::GraphZoneRI& graph_slam, AASS::RSI::GraphZoneRI& graph_slam_model, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, bool use_chi_square_distance, bool use_bachhatrya_distance, const cv::Mat& graph_slam_segmented, const cv::Mat& graph_slam_segmented_model) {


	for(auto vp = boost::vertices(graph_slam); vp.first != vp.second; ++vp.first) {
		AASS::RSI::GraphZoneRI::VertexZoneRI v = *vp.first;
		double size_classification = (graph_slam)[v].getSizeClassification();
		std::cout << "-> " <<size_classification << " ";
	}
	std::cout << "\nsecond graph " << std::endl;
	for(auto vp = boost::vertices(graph_slam_model); vp.first != vp.second; ++vp.first) {
		AASS::RSI::GraphZoneRI::VertexZoneRI v = *vp.first;
		double size_classification = (graph_slam_model)[v].getSizeClassification();
		std::cout << "-> " <<size_classification << " ";
	}
	std::cout << "\ndone " << std::endl;

//	int aaa;
//	std::cin >> aaa;

	graph_slam.setSDAwayFromMeanForUniqueness(1);
	graph_slam_model.setSDAwayFromMeanForUniqueness(1);

	/********** Uniqueness *******************************************/
	///ATTENTION NEED TO CHANGE FOR SKETCH TODO TODO
	if(!is_sketch) {
		graph_slam.updateUnique(graph_slam_model);
		graph_slam_model.updateUnique();
	}
	else{
		graph_slam.updateUnique();
		graph_slam_model.updateUnique();
	};

	/********** Hungarian matching of graph onto itself***************/

	std::cout << "Hungarian Match" << std::endl;
	AASS::RSI::HungarianMatcher hungmatch;
	std::vector<int> scores;
	auto match = hungmatch.match(graph_slam, graph_slam_model, scores);

// 	exit(0);

	std::sort(match.begin(), match.end(), [&graph_slam, &graph_slam_model](AASS::RSI::ZoneCompared &match, AASS::RSI::ZoneCompared &match1){
        assert(match.getSimilarity() != -1);
        assert(match1.getSimilarity() != -1);
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

	for(size_t i = 0 ; i <= match.size() / 2 ; ++i){

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
		double factor = 0;
//		double factor_model = 1;

		if(use_hausdorff_distance){
			std::cout << "Haus" << std::endl;
			factor = std::exp(-gp_laplacian->getHausdorffDistanceRelativeSize(*gp_laplacian_model) );
//			factor_model = graph_slam_model.getHausdorffDistanceRelativeSize();
		}
		else if(use_l2_norm){
			std::cout << "l2" << std::endl;
			factor = std::exp(-gp_laplacian->getMeanL2Norm(*gp_laplacian_model) );
//			factor_model = graph_slam_model.getHausdorffDistanceRelativeSize();
		}
		else if(use_chi_square_distance){
			std::cout << "Chi" << std::endl;
			factor = 1 - gp_laplacian->getChiSquare(*gp_laplacian_model);
//			factor = (1 / std::sqrt(2 * 3.1415)) * std::exp(- factor * factor /2);

		}
		else if(use_bachhatrya_distance){
			std::cout << "Bach" << std::endl;
			factor = 1 - gp_laplacian->getBhattacharyyaDistance(*gp_laplacian_model);
		}

		assert(factor >= 0);
		assert(factor <= 1);

//		std::cout << "Factor " << factor << std::endl;
//		int aaaaa;
//		std::cin>>aaaaa;

		gp_laplacian->useRelativeSizeAsWeights(factor);
		gp_laplacian_model->useRelativeSizeAsWeights(factor);
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


auto create_graphs_laplacian(const std::string& map_input, const std::string& map_model, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, bool use_chi_square_distance, bool use_bachhatrya_distance){

	AASS::RSI::GraphZoneRI graph_slam;
	cv::Mat graph_slam_segmented = makeGraph(map_input, graph_slam);
	AASS::RSI::GraphZoneRI graph_slam_model;
	cv::Mat graph_slam_segmented_model = makeGraph(map_model, graph_slam_model);

	return create_graphs_laplacian(graph_slam, graph_slam_model, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm,use_chi_square_distance, use_bachhatrya_distance, graph_slam_segmented, graph_slam_segmented_model);

}



auto match_maps_vfl(const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& graph_input, const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& graph_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {

	auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(*(std::get<1>(graph_input)), *(std::get<1>(graph_model)), use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm, false, false, std::get<2>(graph_input), std::get<2>(graph_model));

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
    double time_from = 0;
    double time_to = 2;
    double time_step = 0.5;
 	for(double threshold_m = 0 ; threshold_m <= 0.5 && aninput == 0; threshold_m =  threshold_m + 0.05) {

        gp_laplacian->setThrehsoldSameVertices(threshold_m);
        gp_laplacian_model->setThrehsoldSameVertices(threshold_m);
        
        gp_laplacian->test_check_threshold();
        gp_laplacian_model->test_check_threshold();
    
		gp_laplacian->propagateHeatKernel(time_from, time_to, time_step);
		gp_laplacian_model->propagateHeatKernel(time_from, time_to, time_step);
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

            
            //Convert hyp to MatchLaplacian
            AASS::graphmatch::HypotheseLaplacian hyp_lap;
            for(auto &el : hyp_vfl.getAllElements()){
                auto v_input = equivalent_laplacian[el.getFirst()];
                auto v_model = equivalent_laplacian_model[el.getSecond()];
                AASS::graphmatch::MatchLaplacian match(v_input, v_model);
                hyp_lap.push_back(match);
            }
            
            assert(hyp_lap.size() == hyp_vfl.size());


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

			auto [tp, fp, fn, prec, rec, F1] = ev.evaluate(hyp_lap, *gp_laplacian, *gp_laplacian_model);

			if (F1_good == -1 || F1 > F1_good) {
				F1_good = F1;
				good_time = threshold_m;
				precision = prec;
				recall = rec;
				tp_good = tp;
				fp_good = fp;
				fn_good = fn;
			}

			if(all_results.find( threshold_m ) != all_results.end() ) {
				auto data = all_results.find(threshold_m)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(threshold_m)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold_m);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold_m, data));
			}
		}
		else{
            if(all_results.find( threshold_m ) != all_results.end() ) {
				auto data = all_results.find(threshold_m)->second;
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.find(threshold_m)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold_m);
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold_m, data));
			}
        }

		std::cout << "Running on :\n" << gt_file << " with time " << time_from << " res " << tp_good << " " << F1_good << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

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


auto match_maps_hungarian(const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& graph_input, const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& graph_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {

	auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(*(std::get<1>(graph_input)), *(std::get<1>(graph_model)), use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm, false, false, std::get<2>(graph_input), std::get<2>(graph_model));

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

// 	for(double time = 0 ; time <= 10 && aninput == 0; time =  time + 0.5) {
    double time_from = 0;
    double time_to = 2;
    double time_step = 0.5;
    for(double threshold_m = 0 ; threshold_m <= 0.5 && aninput == 0; threshold_m =  threshold_m + 0.05) {

        gp_laplacian->setThrehsoldSameVertices(threshold_m);
        gp_laplacian_model->setThrehsoldSameVertices(threshold_m);
        
        gp_laplacian->test_check_threshold();
        gp_laplacian_model->test_check_threshold();
        
		gp_laplacian->propagateHeatKernel(time_from, time_to, time_step);
		gp_laplacian_model->propagateHeatKernel(time_from, time_to, time_step);
// 		gp_laplacian->propagateHeatKernel(time, time, 0.5);
// 		gp_laplacian_model->propagateHeatKernel(time, time, 0.5);

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
				good_time = threshold_m;
				precision = prec;
				recall = rec;
				tp_good = tp;
				fp_good = fp;
				fn_good = fn;
			}
			if(all_results.find( threshold_m ) != all_results.end() ) {
				auto data = all_results.find(threshold_m)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(threshold_m)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold_m);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold_m, data));
			}
		}
		else{
            if(all_results.find( threshold_m ) != all_results.end() ) {
				auto data = all_results.find(threshold_m)->second;
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.find(threshold_m)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold_m);
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold_m, data));
			}
        }
//		else{
//
//			all_results.push_back( std::make_tuple(tp, fp, fn, prec, rec, F1) );
//		}


		std::cout << "Running on :\n" << gt_file << " with time " << threshold_m << " res " << tp_good << " " << F1_good << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

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



auto match_maps_and_find_time(const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& graph_input, const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& graph_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, bool use_chi_square_distance, bool use_bachhatrya_distance, bool init_with_anchors, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {


	auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(*(std::get<1>(graph_input)), *(std::get<1>(graph_model)), use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm, use_chi_square_distance, use_bachhatrya_distance, std::get<2>(graph_input), std::get<2>(graph_model));

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
// 	for(double time = 0 ; time <= 10 && aninput == 0; time = time + 0.5) {
    double time_from = 0;
    double time_to = 2;
    double time_step = 0.5;
    
    for(double threshold_m = 0 ; threshold_m <= 0.5 && aninput == 0; threshold_m =  threshold_m + 0.025) {

        gp_laplacian->setThrehsoldSameVertices(threshold_m);
        gp_laplacian_model->setThrehsoldSameVertices(threshold_m);
        
        gp_laplacian->test_check_threshold();
        gp_laplacian_model->test_check_threshold();
    
		gp_laplacian->propagateHeatKernel(time_from, time_to, time_step);
		gp_laplacian_model->propagateHeatKernel(time_from, time_to, time_step);        
        
        
// 		gp_laplacian->propagateHeatKernel(time, time, 0.5);
// 		gp_laplacian_model->propagateHeatKernel(time, time, 0.5);

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
        
        if(init_with_anchors){
            auto gp_anchors = gp_laplacian->getAnchors();
            auto gp_anchors_model = gp_laplacian_model->getAnchors();
            
            AASS::graphmatch::HypotheseLaplacian starting_seeds;
            for(int  i = 0 ; i < gp_anchors.size() ; ++i){
                starting_seeds.push_back(AASS::graphmatch::MatchLaplacian(gp_anchors[i], gp_anchors_model[i]) );
            }
            
            graphmatch_custom.planarEditDistanceAlgorithm(starting_seeds, *gp_laplacian, *gp_laplacian_model);
            
        }
        else{
            graphmatch_custom.planarEditDistanceAlgorithm(*gp_laplacian, *gp_laplacian_model);
        }
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
//			std::cout << "Input 0 if not good and anything otherwise" << std::endl;
//			std::cin >> aninput;


			std::cout << "Read file " << gt_file << std::endl;
			AASS::graphmatch::evaluation::Evaluation ev;
			ev.read_file(gt_file);
			std::cout << "Read file" << std::endl;

			auto [tp, fp, fn, prec, rec, F1] = ev.evaluate(hypothesis_final_custom[0], *gp_laplacian, *gp_laplacian_model);

			if (F1_good == -1 || F1 > F1_good) {
				F1_good = F1;
				good_time = threshold_m;
				precision = prec;
				recall = rec;
				tp_good = tp;
				fp_good = fp;
				fn_good = fn;
			}

			if(all_results.find( threshold_m ) != all_results.end() ) {
				auto data = all_results.find(threshold_m)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(threshold_m)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold_m);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold_m, data));
			}
		}
		else{
            if(all_results.find( threshold_m ) != all_results.end() ) {
				auto data = all_results.find(threshold_m)->second;
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.find(threshold_m)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold_m);
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold_m, data));
			}
        }

		std::cout << "Running on :\n" << gt_file << " with time " << threshold_m << " res " << tp_good << " " << F1_good << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " " << gp_laplacian_model->getNumVertices() << std::endl;

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
		myfile << "# threshold_m meantp std meanfp std meanfn std meanprecision std meanrecall std meanF1 std\n";
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
		myfile << "# map tp fp fn precision recall F1 threshold_m\n";
	} else {
		myfile.open(result_file, std::ios::out | std::ios::app);
		myfile << "# map tp fp fn precision recall F1 threshold_m\n";
	}

	if (myfile.is_open()) {
        int count = 0;
		for(auto element : all_results_out) {
            double eee = element.first * 10;
			if( count == 0 ){
				myfile << "# element " << eee << " " << (int)(eee + 0.75) << "\n";
				element.second.export_detailed_data(myfile);
                count ++;
			}
			else{
                count = 0;
            }
		}
		myfile << "\n\n\n";
        count = 0;
		for(auto element : all_results_out) {
            double eee = element.first * 10;
			if( count == 1 ){
				myfile << "# element " << eee << " " << (int)(eee + 0.75) << "\n";
				element.second.export_detailed_data(myfile);
			count ++;
			}
			else{
                count = 1;
            }
		}
	}


}


auto evaluate_all_files(const std::vector<std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat> >& names_maps_images, const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& gt, const std::string gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, bool use_chi_square_distance, bool use_bachhatrya_distance, bool init_with_anchors, const std::string& prefix_details){


	std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results_detailed;

	for(auto element : names_maps_images){

		std::string map_name = std::get<0>(element);

		std::string gt_name = "gt_" + map_name + "_model_simple.dat";
		std::string gt_file = gt_folder + "/" + gt_name;
//		std::cout << "Running on :\n" << map_name << "\nand \n" << input_folder + "/model_simple.png" << std::endl;

		auto[tp, fp, fn, prec, rec, F1, time] = match_maps_and_find_time(element, gt,
		                                                                 gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm, use_chi_square_distance, use_bachhatrya_distance, init_with_anchors, all_results_detailed);

		results.push_back(std::make_tuple(map_name, tp, fp, fn, prec, rec, F1, time));
	}


//	auto rec = std::experimental::filesystem::directory_iterator(input_folder);
//	int count = 0;
//	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {
//
//		auto p_canon = std::experimental::filesystem::canonical(*p);
//		if(!std::experimental::filesystem::is_directory(p_canon) ){
//			auto input_file_stem =p_canon.stem();
//
//			if(input_file_stem.string().compare("model_simple") != 0) {
//				count++;
//
//				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
//				std::string gt_file = gt_folder + "/" + gt_name;
//
//				std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png"
//				          << std::endl;
//
////				std::map< double, std::tuple<double, double, double, double, double, double> > all_results_tmp;
//				auto[tp, fp, fn, prec, rec, F1, time] = match_maps_and_find_time(p_canon.string(), input_folder + "/model_simple.png",
//				                                            gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, all_results_detailed);
//
//				results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));
////				fuse_detailed_results(all_results_tmp, all_results_detailed);
//			}
//
//		}
//
//	}
	export_mean_std_detailed_results(all_results_detailed, "multiple_times_export_detailed_" + prefix_details +".dat");
	export_mean_std_detailed_results_mean(all_results_detailed, "multiple_times_export_detailed_" + prefix_details +"_all_summarized.dat");
	return results;


}

auto evaluate_all_files_hungarian(const std::vector<std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat> >& names_maps_images, const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& gt, const std::string& gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, const std::string& prefix_details){


	std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results_detailed;

	for(auto element : names_maps_images){

		std::string map_name = std::get<0>(element);

		std::string gt_name = "gt_" + map_name + "_model_simple.dat";
		std::string gt_file = gt_folder + "/" + gt_name;
//		std::cout << "Running on :\n" << map_name << "\nand \n" << input_folder + "/model_simple.png" << std::endl;

		auto[tp, fp, fn, prec, rec, F1, time] = match_maps_hungarian(element, gt,
		                                                                 gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm, all_results_detailed);

		results.push_back(std::make_tuple(map_name, tp, fp, fn, prec, rec, F1, time));
	}




//	auto rec = std::experimental::filesystem::directory_iterator(input_folder);
//
//	int count = 0;
//
//
//	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {
//
//		auto p_canon = std::experimental::filesystem::canonical(*p);
//		if(!std::experimental::filesystem::is_directory(p_canon) ){
//			auto input_file_stem =p_canon.stem();
//
//			if(input_file_stem.string().compare("model_simple") != 0) {
//
//				count++;
//
//				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
//				std::string gt_file = gt_folder + "/" + gt_name;
//
//				std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png"
//				          << std::endl;
//
////				std::map< double, std::tuple<double, double, double, double, double, double> > all_results_tmp;
//				auto[tp, fp, fn, prec, rec, F1, time] = match_maps_hungarian(p_canon.string(), input_folder + "/model_simple.png",
//				                                                             gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, all_results_detailed);
//
//				results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));
//
////				fuse_detailed_results(all_results_tmp, all_results_detailed);
//
//			}
//
//		}
//
//	}

	export_mean_std_detailed_results(all_results_detailed, "multiple_times_export_detailed_hungarian_" + prefix_details +".dat");
	export_mean_std_detailed_results_mean(all_results_detailed, "multiple_times_export_detailed_hungarian_" + prefix_details +"_all_summarized.dat");

	return results;


}


auto evaluate_all_files_vfl(const std::vector<std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat> >& names_maps_images, const std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat>& gt, const std::string& gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, bool use_hausdorff_distance, bool use_l2_norm, const std::string& prefix_details){

	std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results_detailed;

	for(auto element : names_maps_images){

		std::string map_name = std::get<0>(element);

		std::string gt_name = "gt_" + map_name + "_model_simple.dat";
		std::string gt_file = gt_folder + "/" + gt_name;
//		std::cout << "Running on :\n" << map_name << "\nand \n" << input_folder + "/model_simple.png" << std::endl;

		auto[tp, fp, fn, prec, rec, F1, time] = match_maps_vfl(element, gt,
		                                                             gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, use_hausdorff_distance, use_l2_norm, all_results_detailed);

		results.push_back(std::make_tuple(map_name, tp, fp, fn, prec, rec, F1, time));
	}


//	int count = 0;
//
//
//	auto rec = std::experimental::filesystem::directory_iterator(input_folder);
//	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {
//
//		auto p_canon = std::experimental::filesystem::canonical(*p);
//		if(!std::experimental::filesystem::is_directory(p_canon) ){
//			auto input_file_stem =p_canon.stem();
//
//			if(input_file_stem.string().compare("model_simple") != 0) {
//				count++;
//
//				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
//				std::string gt_file = gt_folder + "/" + gt_name;
//
//				std::cout << "Running on :\n" << p_canon.string() << "\nand \n" << input_folder + "/model_simple.png"
//				          << std::endl;
//
////				std::map< double, std::tuple<double, double, double, double, double, double> > all_results_tmp;
//				auto[tp, fp, fn, prec, rec, F1, time] = match_maps_vfl(p_canon.string(), input_folder + "/model_simple.png",
//				                                                             gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, all_results_detailed);
//
//				results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));
//
////				fuse_detailed_results(all_results_tmp, all_results_detailed);
//			}
//
//		}
//
//	}
	export_mean_std_detailed_results(all_results_detailed, "multiple_times_export_detailed_vfl_" + prefix_details +".dat");
	export_mean_std_detailed_results_mean(all_results_detailed, "multiple_times_export_detailed_vfl_" + prefix_details +"_all_summarized.dat");
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



auto get_all_images(const std::string& input_folder){

	std::vector<std::tuple<std::string, AASS::RSI::GraphZoneRI*, cv::Mat> > names_graphs_images;

	auto rec = std::experimental::filesystem::directory_iterator(input_folder);
	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {

		auto p_canon = std::experimental::filesystem::canonical(*p);
		if (!std::experimental::filesystem::is_directory(p_canon)) {
			auto input_file_stem = p_canon.stem();

			if (input_file_stem.string().compare("model_simple") != 0) {

//				std::string gt_name = "gt_" + p_canon.stem().string() + "_model_simple.dat";
//				std::string gt_file = gt_folder + "/" + gt_name;
//
				std::cout << "Running on :\n" <<input_file_stem.string() << "\nand \n" << input_folder + "/model_simple.png"
				          << std::endl;

				AASS::RSI::GraphZoneRI* graph_slam = new AASS::RSI::GraphZoneRI();
				cv::Mat graph_slam_segmented = makeGraph(p_canon.string(), *graph_slam);
				names_graphs_images.push_back( std::make_tuple(p_canon.stem().string(), graph_slam, graph_slam_segmented) );
//				AASS::RSI::GraphZoneRI graph_slam_model;
//				cv::Mat graph_slam_segmented_model = makeGraph(map_model, graph_slam_model);


			}
		}
	}
	return names_graphs_images;
}



auto get_gt(const std::string gt_name){

//	std::string gt_file = gt_folder + "/" + gt_name;
	AASS::RSI::GraphZoneRI* graph_slam = new AASS::RSI::GraphZoneRI();
	cv::Mat graph_slam_segmented = makeGraph(gt_name, *graph_slam);

	return std::make_tuple(gt_name, graph_slam, graph_slam_segmented);

}

int main(int argc, char** argv){


    bool use_hungarian = false;
    bool use_vfl = false;
    bool use_planar = true;
    bool use_old_method = true;
    bool evaluate_human_vs_human = false;
    bool evaluate_algo_vs_human = true;
    bool use_sketches = true;
    bool use_robot_maps = false;


    std::vector< std::tuple<std::string, std::string, std::string > > input_gt_name;
    std::vector< std::tuple<std::string, std::string, std::string, std::string > > human_vs_human;
    
    if(use_robot_maps){
        input_gt_name.push_back( std::make_tuple("../../../../Test/RSI/RobotMaps/E5/Malcolm" , "../../../../Test/RSI/RobotMaps/E5/Malcolm/GT", "E5_malcolm_") );
        input_gt_name.push_back( std::make_tuple("../../../../Test/RSI/RobotMaps/E5/Vasiliki" , "../../../../Test/RSI/RobotMaps/E5/Vasiliki/GT", "E5_vasiliki_") );
        //NOT THE SAME GT MAP SO NOT COMPARABLE
//         human_vs_human.push_back( std::make_tuple("../../../../Test/RSI/RobotMaps/E5/Vasiliki", "../../../../Test/RSI/RobotMaps/E5/Malcolm/GT", "../../../../Test/RSI/RobotMaps/E5/Vasiliki/GT", "E5_") );
    }
    if(use_sketches){
        input_gt_name.push_back( std::make_tuple("../../../../Test/RSI/Sketches/" , "../../../../Test/RSI/Sketches/GT/Malcolm", "sketches_malcolm_") );
        input_gt_name.push_back( std::make_tuple("../../../../Test/RSI/Sketches/" , "../../../../Test/RSI/Sketches/GT/Vasiliki", "sketches_vasiliki_") );
        human_vs_human.push_back( std::make_tuple("../../../../Test/RSI/Sketches/" , "../../../../Test/RSI/Sketches/GT/Malcolm", "../../../../Test/RSI/Sketches/GT/Vasiliki", "sketches_") );
    }


    if(evaluate_algo_vs_human){
        for(auto element : input_gt_name){
        
            auto [input_folder, gt_folder, name] = element;

            auto names_graphs_images = get_all_images(input_folder);
            auto gt = get_gt(gt_folder + "/model_simple.png");

            if(use_old_method){
                auto results_base_old_method =  evaluate_all_files(names_graphs_images, gt, gt_folder, false, false, false, true, false, false, false, false, false, name + "base_old");
                std::cout << "Results base_old_method" << std::endl;
                print_results(results_base_old_method);
                export_results("multiple_times_" + name + "results_base_old_method.dat", results_base_old_method);
                
            }

        //	auto results_anchors_old_method =  evaluate_all_files(input_folder, gt_folder, true, false, false, true, "anchors_old");
        //	std::cout << "Results Anchors_old_method" << std::endl;
        //	print_results(results_anchors_old_method);
        //	export_results("results_anchors_old_method.dat", results_anchors_old_method);
        //
        //	auto results_anchors_uniqueness_old_method =  evaluate_all_files(input_folder, gt_folder, true, true, false, true, "anchors_unique_old");
        //	auto results_anchors_relative_size_old_method =  evaluate_all_files(input_folder, gt_folder, true, false, true, true, "anchors_relative_size_old");
        //
        //	std::cout << "Results Anchors Uniqueness_old_method" << std::endl;
        //	print_results(results_anchors_uniqueness_old_method);
        //	export_results("results_anchors_uniqueness_old_method.dat", results_anchors_uniqueness_old_method);
        //	std::cout << "Results Anchors Relative size_old_method" << std::endl;
        //	print_results(results_anchors_relative_size_old_method);
        //	export_results("results_anchors_relative_size_old_method.dat", results_anchors_relative_size_old_method);


            if(use_planar){
               auto results_base =  evaluate_all_files(names_graphs_images, gt, gt_folder, false, false, false, false, false, false, false, false, false, name +  "base");
//                auto results_base_init_seeds =  evaluate_all_files(names_graphs_images, gt, gt_folder, false, false, false, false, false, false, false, false, true, name +  "base_init_seeds");
               auto results_anchors =  evaluate_all_files(names_graphs_images, gt, gt_folder, true, false, false, false, false, false, false, false, false, name +  "anchors");
               auto results_relative_size =  evaluate_all_files(names_graphs_images, gt, gt_folder, false, false, true, false, false, false, false, false, false, name + "anchors_relative_size");
               auto results_anchors_relative_size =  evaluate_all_files(names_graphs_images, gt, gt_folder, true, false, true, false, false, false, false, false, false, name + "anchors_relative_size");
//                 auto results_anchors_relative_size_haus =  evaluate_all_files(names_graphs_images, gt, gt_folder, true, false, true, false, true, false, false, false, "anchors_relative_size_hausdorff");
//                 auto results_anchors_relative_size_l2 =  evaluate_all_files(names_graphs_images, gt, gt_folder, true, false, true, false, false, true, false, false, "anchors_relative_size_l2");
//                 auto results_anchors_relative_size_chisquare =  evaluate_all_files(names_graphs_images, gt, gt_folder, true, false, true, false, false, false, true, false, "anchors_relative_size_chisquare");
//                 auto results_anchors_relative_size_chadist =  evaluate_all_files(names_graphs_images, gt, gt_folder, true, false, true, false, false, false, false, true, "anchors_relative_size_chadist");

               std::cout << "Results base" << std::endl;
               print_results(results_base);
               export_results("multiple_times_" +name + "results_base.dat", results_base);
               std::cout << "Results base" << std::endl;
//                print_results(results_base_init_seeds);
//                export_results("multiple_times_" +name + "results_base_init_seeds.dat", results_base_init_seeds);
               std::cout << "Results Anchors" << std::endl;
               print_results(results_anchors);
               export_results("multiple_times_" +name + "results_anchors.dat", results_anchors);
               std::cout << "Results Anchors Uniqueness" << std::endl;
               print_results(results_relative_size);
               export_results("multiple_times_" +name + "results_relative_size.dat", results_relative_size);
               std::cout << "Results Anchors Relative size" << std::endl;
               print_results(results_anchors_relative_size);
              export_results("multiple_times_" + name + "results_anchors_relative_size.dat", results_anchors_relative_size);
//                 std::cout << "Results Anchors Relative size haus" << std::endl;
//                 print_results(results_anchors_relative_size_haus);
//                 export_results("results_anchors_relative_size_haus.dat", results_anchors_relative_size_haus);
//                 std::cout << "Results Anchors Relative size l2" << std::endl;
//                 print_results(results_anchors_relative_size_l2);
//                 export_results("results_anchors_relative_size_l2.dat", results_anchors_relative_size_l2);
//                 std::cout << "Results Anchors Relative size chisquare" << std::endl;
//                 print_results(results_anchors_relative_size_chisquare);
//                 export_results("results_anchors_relative_size_chisquare.dat", results_anchors_relative_size_chisquare);
//                 std::cout << "Results Anchors Relative size chadist" << std::endl;
//                 print_results(results_anchors_relative_size_chadist);
//                 export_results("results_anchors_relative_size_chadist.dat", results_anchors_relative_size_chadist);
            
            }

            if(use_hungarian){
                auto results_base_hungarian =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, false, false, false, false, false, false,name +  "base");
                auto results_anchors_hungarian =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, true, false, false, false, false, false, name + "anchors");
                auto results_anchors_relative_size_hungarian =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, true, false, true, false, false, false, name + "anchors_unique");
                auto results_relative_size_hungarian =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, false, false, true, false, false, false, name + "anchors_relative_size");
        //     auto results_anchors_relative_size_hungarian_haus =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, true, false, true, false, true, false, name + "anchors_relative_size_hausdorff");
        //     auto results_anchors_relative_size_hungarian_l2 =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, true, false, true, false, false, true, name + "anchors_relative_size_l2");
                auto results_old_method_hungarian =  evaluate_all_files_hungarian(names_graphs_images, gt, gt_folder, true, false, true, true, false, false, name + "old_method");

                std::cout << "Results base_hungarian" << std::endl;
                print_results(results_base_hungarian);
                export_results("multiple_times_" + name + "results_base_hungarian.dat", results_base_hungarian);
                std::cout << "Results Anchors_hungarian" << std::endl;
                print_results(results_anchors_hungarian);
                export_results("multiple_times_" + name + "results_anchors_hungarian.dat", results_anchors_hungarian);
                std::cout << "Results Anchors Uniqueness_hungarian" << std::endl;
                print_results(results_anchors_results_relative_size_hungarian);
                export_results("multiple_times_" + name + "results_anchors_results_relative_size_hungarian.dat", results_anchors_results_relative_size_hungarian);
                std::cout << "Results Anchors Relative size_hungarian" << std::endl;
                print_results(results_anchors_relative_size_hungarian);
                export_results("multiple_times_" + name + "results_anchors_relative_size_hungarian.dat", results_anchors_relative_size_hungarian);
    //	        std::cout << "Results Anchors Relative size_hungarian_haus" << std::endl;
    //	        print_results(results_anchors_relative_size_hungarian_haus);
    //	        export_results(name + "results_anchors_relative_size_hungarian_haus.dat", results_anchors_relative_size_hungarian_haus);
    //	        std::cout << "Results Anchors Relative size_hungarian_l2" << std::endl;
    //	        print_results(results_anchors_relative_size_hungarian_l2);
    //	        export_results(name + "results_anchors_relative_size_hungarian_l2.dat", results_anchors_relative_size_hungarian_l2);
                std::cout << "Results Old Method hungarian" << std::endl;
                print_results(results_old_method_hungarian);
                export_results("multiple_times_" + name + "results_old_method_hungarian.dat", results_old_method_hungarian);
            }

            if(use_vfl){
                auto results_base_vfl =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, false, false, false, false, false, false, name + "base");
                auto results_anchors_vfl =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, true, false, false, false, false, false, name + "anchors");
                auto results_anchors_results_relative_size_vfl =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, false, false, true, false, false, false, name + "anchors_unique");
                auto results_anchors_relative_size_vfl =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, true, false, true, false, false, false, name + "anchors_relative_size");
    //	        auto results_anchors_relative_size_vfl_haus =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, true, false, true, false, true, false, name + "anchors_relative_size_hausdorff");
    //	        auto results_anchors_relative_size_vfl_l2 =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, true, false, true, false, false, true, name + "anchors_relative_size_l2");
                auto results_old_method_vfl =  evaluate_all_files_vfl(names_graphs_images, gt, gt_folder, true, false, true, true, false, false, name + "old_method");


                std::cout << "Results base_vfl" << std::endl;
                print_results(results_base_vfl);
                export_results("multiple_times_" + name + "results_base_vfl.dat", results_base_vfl);
                std::cout << "Results Anchors_vfl" << std::endl;
                print_results(results_anchors_vfl);
                export_results("multiple_times_" + name + "results_anchors_vfl.dat", results_anchors_vfl);
                std::cout << "Results Anchors Uniqueness_vfl" << std::endl;
                print_results(results_anchors_results_relative_size_vfl);
                export_results("multiple_times_" + name + "results_anchors_results_relative_size_vfl.dat", results_anchors_results_relative_size_vfl);
                std::cout << "Results Anchors Relative size_vfl" << std::endl;
                print_results(results_anchors_relative_size_vfl);
                export_results("multiple_times_" + name + "results_anchors_relative_size_vfl.dat", results_anchors_relative_size_vfl);
    //	        std::cout << "Results Anchors Relative size_vfl_haus" << std::endl;
    //	        print_results(results_anchors_relative_size_vfl_haus);
    //	        export_results(name + "results_anchors_relative_size_vfl_haus.dat", results_anchors_relative_size_vfl_haus);
    //	        std::cout << "Results Anchors Relative size_vfl_l2" << std::endl;
    //	        print_results(results_anchors_relative_size_vfl_l2);
    //	        export_results(name + "results_anchors_relative_size_vfl_l2.dat", results_anchors_relative_size_vfl_l2);
                std::cout << "Results Old Method vfl" << std::endl;
                print_results(results_old_method_vfl);
                export_results("multiple_times_" + name + "results_old_method_vfl.dat", results_old_method_vfl);

            }
            //Evaluate threshold

            
        }
    }
    
    if( evaluate_human_vs_human == true){
        for(auto element : human_vs_human){
        
            auto [input_folder, gt_folder_input, gt_folder_model, name] = element;
            auto names_graphs_images = get_all_images(input_folder);
            auto gt = get_gt(gt_folder_model + "/model_simple.png");
            
            
            std::vector<std::tuple<std::string, double, double, double, double, double, double, double > > results;
            
            for(auto element_name : names_graphs_images){
                
                cv::Mat graph_slam_segmented, graph_slam_segmented_model;
                
                AASS::RSI::GraphZoneRI* gz = std::get<1>(element_name);
                AASS::RSI::GraphZoneRI* gz_gt = std::get<1>(gt);
                
                auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented2, graph_slam_segmented_model2] = create_graphs_laplacian(*gz, *gz_gt, false, false, false, false, false, false, false, false, graph_slam_segmented, graph_slam_segmented_model);
                
                std::string map_name = std::get<0>(element_name);
                std::string gt_name = "gt_" + map_name + "_model_simple.dat";
                
                std::string gt_file_input = gt_folder_input + "/" + gt_name;
                std::string gt_file_model = gt_folder_model + "/" + gt_name;
                
                std::cout << "Name " << name << " Gts input : " << gt_file_input << "\ngot file model " << gt_file_model << std::endl;
                
                gp_laplacian->propagateHeatKernel(5, 5, 0);
                gp_laplacian_model->propagateHeatKernel(5, 5, 0);
                
//                 cv::Mat zone_img = cv::Mat::zeros(1500, 1500, CV_8UC1);
//                 gp_laplacian->drawSpecial(zone_img);
//                 cv::imshow ("GP laplacian", zone_img);
//                 cv::Mat zone_imgm = cv::Mat::zeros(1500, 1500, CV_8UC1);
//                 gp_laplacian_model->drawSpecial(zone_imgm);
//                 cv::imshow ("GP laplacian Model", zone_imgm);
//                 cv::waitKey(0);
                
                AASS::graphmatch::evaluation::CreateHypFromGT createhypgt;
                auto [tp, fp, fn, precision, recall, F1] = createhypgt.compare(gt_file_input, gt_file_model, *gp_laplacian, *gp_laplacian_model );
                
                results.push_back(std::make_tuple(map_name, tp, fp, fn, precision, recall, F1, 0) );
            
            }
            export_results("multiple_times_" + name + "results_human_vs_human.dat", results);
        }
 
    }


}

