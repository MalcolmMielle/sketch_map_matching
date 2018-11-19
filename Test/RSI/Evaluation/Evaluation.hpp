#ifndef SKETCHALGORIHTM_EVALUATION_05112018
#define SKETCHALGORIHTM_EVALUATION_05112018

#include <iostream>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <sstream>
#include <vector>
#include <string>
#include <array>
#include <experimental/array>

#include <tuple>

#include <bettergraph/HypotheseBase.hpp>

#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/HypotheseLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"

#include "MatchMaps.hpp"

namespace AASS{
	namespace graphmatch{
		namespace evaluation{


			class DataEvaluation{

			public:

				double time;

				std::vector<std::array<double, 6> > tp_fp_fn_precision_recall_F1;

				DataEvaluation(double time_t) : time(time_t){};

				void push_back(const std::tuple<double, double, double, double, double, double>& input){
					auto array = std::experimental::make_array(std::get<0>(input), std::get<1>(input), std::get<2>(input), std::get<3>(input), std::get<4>(input), std::get<5>(input));
					tp_fp_fn_precision_recall_F1.push_back(array);
				}

				auto getMeanStdTp() const {
					return std::make_tuple(getMean(0), getStd(0) );
				}
				auto getMeanStdFp() const {
					return std::make_tuple(getMean(1), getStd(1) );
				}
				auto getMeanStdFn() const {
					return std::make_tuple(getMean(2), getStd(2) );
				}
				auto getMeanStdPrecision() const {
					return std::make_tuple(getMean(3), getStd(3) );
				}
				auto getMeanStdRecall() const {
					return std::make_tuple(getMean(4), getStd(4) );
				}
				auto getMeanStdF1() const {
					return std::make_tuple(getMean(5), getStd(5) );
				}

				void export_mean_data(std::ofstream& myfile) const {

					auto meanstdtp = getMeanStdTp();
					auto meanstdfp = getMeanStdFp();
					auto meanstdfn = getMeanStdFn();
					auto meanstdprecision = getMeanStdPrecision();
					auto meanstdrecall = getMeanStdRecall();
					auto meanstdF1 = getMeanStdF1();
					myfile << time << " " << std::get<0>(meanstdtp) << " " << std::get<1>(meanstdtp) << " " << std::get<0>(meanstdfp) << " " << std::get<1>(meanstdfp) << " " << std::get<0>(meanstdfn) << " " << std::get<1>(meanstdfn) << " " << std::get<0>(meanstdprecision) << " " << std::get<1>(meanstdprecision) << " " << std::get<0>(meanstdrecall) << " " << std::get<1>(meanstdrecall) << " " << std::get<0>(meanstdF1) << " " << std::get<1>(meanstdF1) << "\n" ;


				}

				void export_detailed_data(std::ofstream& myfile) const {

					myfile << "#time tp fp fn precision recall F1\n";
					for (auto element : tp_fp_fn_precision_recall_F1) {
						myfile << time << " " << element[0] << " " << element[1] << " " << element[2] << " "
						       << element[3] << " " << element[4] << " " << element[5] << "\n";
					}
				}

				void export_data(std::ofstream& myfile) const {

					export_detailed_data(myfile);
					myfile << "\n";

					myfile << "# time meantp std meanfp std meanfn std meanprecision std meanrecall std meanF1 std\n";
					export_mean_data(myfile);

					myfile << "\n\n";


				}

			private:

				double getMean(int index) const {
					double sum = 0;
					for(auto element : tp_fp_fn_precision_recall_F1){
						sum += element[index];
					}
					return sum / (double) tp_fp_fn_precision_recall_F1.size();
				}

				double getStd(int index) const {
					double mean_t = getMean(index);
					double std_sum = 0;
					for (auto element : tp_fp_fn_precision_recall_F1) {
						std_sum += (element[index] - mean_t) * (element[index] - mean_t);
					}
					return std::sqrt(std_sum / (tp_fp_fn_precision_recall_F1.size() - 1 ) );
				}

			};







			class Evaluation{

			protected:

				MatchesBetweenMaps _matches;

			public:
				Evaluation() : _matches("", ""){}

				void read_file(const std::string& file){

					std::cout << "Read from file: " << file << std::endl;

					std::ifstream infile(file);
					std::string line;

					std::getline(infile, line);
					auto words = getWords(line);
					assert(words[0].compare("#") == 0);
					assert(words.size() == 3);
					_matches.map1 = words[1];
					_matches.map2 = words[2];

					std::cout << "Names " << _matches.map1 << " " << _matches.map2 << std::endl;

					int count = 0;

					MatchMaps match(_matches.map1, _matches.map2);
					while (std::getline(infile, line))
					{

						auto words_tmp = getWords(line);
						if(words_tmp.size() > 0) {
							if (words_tmp[0].compare("#") == 0) {
//								std::cout << "# " << count << std::endl;
								++count;
								if (count == 4) {
									count = 1;
									_matches.matches.push_back(match);
									match.clear();
								}
							}
							if (words_tmp[0].compare("#") != 0) {
//								std::cout << "Not# " << count << std::endl;

								if (count == 2) {
//									std::cout << "ONE" << std::endl;
									match.read(words_tmp, 0);
								}
								else if (count == 3) {
//									std::cout << "TWO" << std::endl;
									match.read(words_tmp, 1);
								}
								else{
//									std::cout << "Count " << count << std::endl;
									throw std::runtime_error("reach not reachable point");
								}

							}
						}

					}

					//Pushing the last one
					_matches.matches.push_back(match);
					match.clear();


					_matches.print();

				}


				auto evaluate(const bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region*> >& hyp, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					std::deque<bettergraph::MatchComparable<AASS::graphmatch::Region*> > tp_list;
					for(auto match : hyp.getAllElements()){
						if(is_correct(*(match.getFirst()), *(match.getSecond() ) ) ){
							tp_list.push_back(match);
						}
					}

					std::cout << "tp: " << tp_list.size() << std::endl;

					double count = 0;
					for(auto match : hyp.getAllElements()){
						auto v1 = match.getFirst();
						auto v2 = match.getSecond();
						for(auto match_found : tp_list){
							auto v1_found = match_found.getFirst();
							auto v2_found = match_found.getSecond();

							if(v1 == v1_found || v1 == v2_found || v2 == v1_found || v2 == v2_found){
								count ++;
							}
						}
					}

					std::cout << "done with count: " << count << std::endl;

					assert(count >= tp_list.size());
					double fp = hyp.getAllElements().size() - count;
					double tp = tp_list.size();
					double fn = fn_from_matches(tp_list, gl, gl_model);
					assert(fn >= 0);

					double precision = tp / (tp + fp);
					double recall = tp / (tp + fn);
					double F_measure = (2 * precision * recall) / (precision + recall);

					if(tp == 0){F_measure = 0;}

//					double mcc = ( (tp * tn) - (fp * fn) ) / ( std::sqrt( (tp + fn) * (tp + fn) * (tn + fp) * (tn + fn) ));

					return std::make_tuple(tp, fp, fp, precision, recall, F_measure);


				}


				auto evaluate(const AASS::graphmatch::HypotheseLaplacian& hyp, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					std::deque<AASS::graphmatch::MatchLaplacian> tp_list;
					for(auto match : hyp.getMatches()){
						if(is_correct(match, gl, gl_model) ){
							tp_list.push_back(match);
						}
					}

					std::cout << "tp: " << tp_list.size() << std::endl;

					double count = 0;
					for(auto match : hyp.getMatches()){
						auto v1 = match.getFirst();
						auto v2 = match.getSecond();
						for(auto match_found : tp_list){
							auto v1_found = match_found.getFirst();
							auto v2_found = match_found.getSecond();

							if(v1 == v1_found || v1 == v2_found || v2 == v1_found || v2 == v2_found){
								count ++;
							}
						}
					}

					std::cout << "done with count: " << count << std::endl;

					assert(count >= tp_list.size());
					double fp = hyp.getMatches().size() - count;
					double tp = tp_list.size();
					double fn = fn_from_matches(tp_list, gl, gl_model);
					assert(fn >= 0);

					double precision = tp / (tp + fp);
					double recall = tp / (tp + fn);
					double F_measure = (2 * precision * recall) / (precision + recall);

					if(tp == 0){F_measure = 0;}

//					double mcc = ( (tp * tn) - (fp * fn) ) / ( std::sqrt( (tp + fn) * (tp + fn) * (tn + fp) * (tn + fn) ));

					return std::make_tuple(tp, fp, fp, precision, recall, F_measure);

				}

				double fn_from_matches(const std::deque<AASS::graphmatch::MatchLaplacian>& tp_list, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){
					double fn = 0;
					for(auto mapmatches : _matches.matches) {
						for (auto point_map1 : mapmatches.pt_map1) {
							for (auto point_map2 : mapmatches.pt_map2) {
								if(fn_from_matches(point_map1, point_map2, tp_list, gl, gl_model)){
									fn ++;
								}
							}
						}
					}
					return fn;
				}

				bool fn_from_matches(const cv::Point2i& map1_point, const cv::Point2i& map2_point, const std::deque<AASS::graphmatch::MatchLaplacian>& tp_list, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					bool is_fn = true;
					for(auto match : tp_list){
						auto v1 = match.getFirst();
						auto v2 = match.getSecond();
						auto region1 = gl[v1];
						auto region_model = gl[v2];

						auto zone = region1.zone.getZone();
						for(auto point : zone ) {
//								std::cout << point.x << " " << point.y  << " and " << point_map1.x << " " << point_map1.y << std::endl;
							if (map1_point == point) {
								std::cout << "Found the point: " << point << std::endl;
								auto zone_model = region_model.zone.getZone();
								//										std::cout << "Searching: " << point_map2 << std::endl;
//										int count = 0;
								for (auto point_model : zone_model) {
//											std::cout << "Searching : " << point_map2 << " " << point_model << std::endl;
									if (map2_point == point_model) {
										//It's a true positive
										is_fn = false;
									}
								}
							}
						}

					}

					return is_fn;

				}


				double fn_from_matches(const std::deque<bettergraph::MatchComparable<AASS::graphmatch::Region*> >& tp_list, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){
					double fn = 0;
					for(auto mapmatches : _matches.matches) {
						for (auto point_map1 : mapmatches.pt_map1) {
							for (auto point_map2 : mapmatches.pt_map2) {
								if(fn_from_matches(point_map1, point_map2, tp_list, gl, gl_model)){
									fn ++;
								}
							}
						}
					}
					return fn;
				}

				bool fn_from_matches(const cv::Point2i& map1_point, const cv::Point2i& map2_point, const std::deque<bettergraph::MatchComparable<AASS::graphmatch::Region*> >& tp_list, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					bool is_fn = true;
					for(auto match : tp_list){
						auto v1 = match.getFirst();
						auto v2 = match.getSecond();
						auto region1 = *v1;
						auto region_model = *v2;

						auto zone = region1.zone.getZone();
						for(auto point : zone ) {
//								std::cout << point.x << " " << point.y  << " and " << point_map1.x << " " << point_map1.y << std::endl;
							if (map1_point == point) {
								std::cout << "Found the point: " << point << std::endl;
								auto zone_model = region_model.zone.getZone();
								//										std::cout << "Searching: " << point_map2 << std::endl;
//										int count = 0;
								for (auto point_model : zone_model) {
//											std::cout << "Searching : " << point_map2 << " " << point_model << std::endl;
									if (map2_point == point_model) {
										//It's a true positive
										is_fn = false;
									}
								}
							}
						}

					}

					return is_fn;

				}


				bool is_correct(const AASS::graphmatch::Region& region1, const AASS::graphmatch::Region& region_model){
					auto zone = region1.zone.getZone();

					for(auto mapmatches : _matches.matches){

						for(auto point_map1 : mapmatches.pt_map1){

//							cv::Scalar scal(255);
//							cv::Scalar color(150);
//							cv::Mat zone_img = cv::Mat::zeros(500, 500, CV_8UC1);
//							region1.zone.drawZone(zone_img, scal);
//
//							cv::circle(zone_img, cv::Point2i(point_map1.y, point_map1.x), 5, color, -1);
//							cv::imshow ("zone test", zone_img);
//							cv::waitKey(0);

							for(auto point : zone ){

//								std::cout << point.x << " " << point.y  << " and " << point_map1.x << " " << point_map1.y << std::endl;

								if(point_map1 == point){
									std::cout << "Found the point: " << point << std::endl;
									auto zone_model = region_model.zone.getZone();

									for(auto point_map2 : mapmatches.pt_map2) {


//										cv::Mat zone_img_map2 = cv::Mat::zeros(500, 500, CV_8UC1);
//										region_model.zone.drawZone(zone_img_map2, scal);
//
//										cv::circle(zone_img_map2, cv::Point2i(point_map2.y, point_map2.x), 5, color, -1);
//										cv::imshow ("Model zone test", zone_img_map2);
//										cv::waitKey(0);

//										std::cout << "Searching: " << point_map2 << std::endl;
//										int count = 0;
										for(auto point_model : zone_model ){
//											std::cout << "Searching : " << point_map2 << " " << point_model << std::endl;
											if (point_map2 == point_model) {
												std::cout << "Found with " << point << " and " << point_model
												          << std::endl;
												return true;
											}
//											++count;
										}
//										std::cout << "Count false pixel : " << count << std::endl;
									}
								}
							}
						}

					}
					std::cout << "Not found" << std::endl;

					return false;


				}

				bool is_correct(const AASS::graphmatch::MatchLaplacian& match, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					auto v1 = match.getFirst();
					auto v2 = match.getSecond();
					std::cout << "Getting regions" << std::endl;
					auto region1 = gl[v1];
					auto region_model = gl[v2];
					std::cout << "DONE  Getting regions" << std::endl;

					return is_correct(region1, region_model);

				}

			private:

				std::vector<std::string> getWords(const std::string& line){

					std::istringstream iss(line);
					char split_char = ' ';
					std::vector<std::string> tokens;
					for (std::string each; std::getline(iss, each, split_char); tokens.push_back(each));

					return tokens;
				}


			};



		}
	}
}


#endif