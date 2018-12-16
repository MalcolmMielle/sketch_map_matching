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




void match_maps(const std::string& map_input, const std::string& map_model, const std::string& gt_file, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, double time, std::map< double, AASS::graphmatch::evaluation::DataEvaluation >& all_results) {


	auto [gp_laplacian, gp_laplacian_model, graph_slam_segmented, graph_slam_segmented_model] = create_graphs_laplacian(map_input, map_model, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme);

	if(use_old_matching_scheme == true) {
		assert(gp_laplacian->isUsingOldMethod());
		assert(gp_laplacian_model->isUsingOldMethod());
	}

	/********** LAPLACIAN FAMILY SIGNATURES ****************/

	gp_laplacian->propagateHeatKernel(time, time, 0.5);
	gp_laplacian_model->propagateHeatKernel(time, time, 0.5);

	/********** GRAPH MATCHING ****************************/

	for(double threshold = 0 ; threshold <= 1; threshold =  threshold + 0.05) {


		gp_laplacian->setThrehsoldSameVertices(threshold);
		gp_laplacian_model->setThrehsoldSameVertices(threshold);

//		double tp = 0, fp = 0, fn = 0, prec = 0, rec = 0, F1 = 0;

		AASS::graphmatch::GraphMatcherNeighborLaplacian graphmatch_custom;
		graphmatch_custom.planarEditDistanceAlgorithm(*gp_laplacian, *gp_laplacian_model);
		std::cout << "DONE" << std::endl;

		std::deque<
				AASS::graphmatch::HypotheseLaplacian
		> hypothesis_final_custom = graphmatch_custom.getResult();

		if (hypothesis_final_custom.size() > 0) {
			std::cout << "Sorting" << std::endl;

			graphmatch_custom.sort(hypothesis_final_custom);
			std::cout << "Sorted" << std::endl;


			std::cout << "Read file " << gt_file << std::endl;
			AASS::graphmatch::evaluation::Evaluation ev;
			ev.read_file(gt_file);
			std::cout << "Read file" << std::endl;

			auto [tp, fp, fn, prec, rec, F1] = ev.evaluate(hypothesis_final_custom[0], *gp_laplacian, *gp_laplacian_model);

			if(all_results.find( threshold ) != all_results.end() ) {
				auto data = all_results.find(threshold)->second;
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.find(threshold)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold);
				data.push_back(std::make_tuple(tp, fp, fn, prec, rec, F1));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold, data));
			}

		}
		else{
			if(all_results.find( threshold ) != all_results.end() ) {
				auto data = all_results.find(threshold)->second;
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.find(threshold)->second = data;
			}
			else{
				AASS::graphmatch::evaluation::DataEvaluation data(threshold);
				data.push_back(std::make_tuple(0, 0, 0, 0, 0, 0));
				all_results.insert(std::pair<double, AASS::graphmatch::evaluation::DataEvaluation>(threshold, data));
			}
		}

//		std::cout << "Running on :\n" << gt_file << " with time " << time << " res " << tp << " " << F1
//		          << ".\nThe graph sizes " << gp_laplacian->getNumVertices() << " "
//		          << gp_laplacian_model->getNumVertices() << std::endl;

	}

	delete gp_laplacian;
	delete gp_laplacian_model;

//	if(F1 == -1){
//		F1 = 0;
//		time = -1;
//	}

//	return std::make_tuple(tp, fp, fn, prec, rec, F1, time);

}



auto evaluate_all_files(const std::string& input_folder, const std::string& gt_folder, bool use_anchor_heat, bool use_uniqueness_score, bool use_relative_size_as_weight, bool use_old_matching_scheme, const std::string& prefix_details, double time_t){

	std::map< double, AASS::graphmatch::evaluation::DataEvaluation > all_results;

//	double time_t = 8;


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
				match_maps(p_canon.string(), input_folder + "/model_simple.png",
				                                                                 gt_file, use_anchor_heat, use_uniqueness_score, use_relative_size_as_weight, use_old_matching_scheme, time_t, all_results);

//				all_results.push_back(std::make_tuple(p_canon.stem().string(), tp, fp, fn, prec, rec, F1, time));
//				fuse_detailed_results(all_results_tmp, all_results_detailed);
			}

		}

	}
//	export_mean_std_detailed_results(all_results_detailed, "export_detailed_" + prefix_details +".dat");
//	export_mean_std_detailed_results_mean(all_results_detailed, "export_detailed_" + prefix_details +"_all_summarized.dat");
	return all_results;


}



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
			if( 10 * element.first == static_cast<int>(10 * element.first)){
				myfile << "# element " << element.first << "\n";
				element.second.export_detailed_data(myfile);
			}
		}
		myfile << "\n\n\n";
		for(auto element : all_results_out) {
			if( 10 * element.first != static_cast<int>(10 * element.first)){
				myfile << "# element " << element.first << "\n";
				element.second.export_detailed_data(myfile);
			}
		}
	}


}







int main(int argc, char** argv) {


	std::string input_folder = "../../../../Test/RSI/Sketches";
	std::string gt_folder = "../../../../Test/RSI/Sketches/GT";

	double time_t = 0.8;
	auto results_anchors_relative_size =  evaluate_all_files(input_folder, gt_folder, true, false, true, false, "anchors_relative_size", time_t);
	export_mean_std_detailed_results(results_anchors_relative_size, "threshold.dat");
	export_mean_std_detailed_results_mean(results_anchors_relative_size, "threshold_mean.dat");


}
