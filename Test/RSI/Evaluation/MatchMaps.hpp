#ifndef EVALUATION_MATCHMAP_04112018
#define EVALUATION_MATCHMAP_04112018

#include <iostream>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
#include <vector>
#include <opencv2/opencv.hpp>

#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/HypotheseLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"

namespace AASS {
	namespace graphmatch {
		namespace evaluation {

			inline bool exists_test3 (const std::string& name) {
				struct stat buffer;
				return (stat (name.c_str(), &buffer) == 0);
			}

			struct MatchMaps {

				MatchMaps(const std::string &name, const std::string &name2) : map1(name), map2(name2) {};

				std::string map1;
				std::string map2;

				std::vector <cv::Point2i> pt_map1;
				std::vector <cv::Point2i> pt_map2;
                
				std::vector <AASS::graphmatch::GraphLaplacian::VertexLaplacian> vertex_map1;
				std::vector <AASS::graphmatch::GraphLaplacian::VertexLaplacian> vertex_map2;

				void print() {
					std::cout << "Point map1 : " << "\n";
					for (auto point : pt_map1) {
						std::cout << point.x << " " << point.y << std::endl;
					}

					std::cout << "Point map2 : " << "\n";
					for (auto point : pt_map2) {
						std::cout << point.x << " " << point.y << std::endl;
					}
					
					std::cout << "\n VERTICES \n" << std::endl;
					for(auto &vet : vertex_map1){
                        std::cout << vet << std::endl;
                    }
                    for(auto &vet2 : vertex_map2){
                        std::cout << vet2 << std::endl;
                        
                    }
				}

				void export_all(std::ofstream &myfile) {
					myfile << "# map1" << "\n";
					for (auto point : pt_map1) {
						myfile << point.x << " " << point.y << std::endl;
					}

					myfile << "# map2" << "\n";
					for (auto point : pt_map2) {
						myfile << point.x << " " << point.y << std::endl;
					}
				}

				void read(const std::vector<std::string>& line, int map){
					cv::Point2i point;
					assert(line.size() == 2);
					point.x = std::stoi( line[0] );
					point.y = std::stoi( line[1] );
					std::cout << "Point " << point << std::endl;
					if(map == 0){
						pt_map1.push_back(point);
					}else{
						pt_map2.push_back(point);
					}
				}

				void clear(){
					pt_map1.clear();
					pt_map2.clear();
                    vertex_map1.clear();
				    vertex_map2.clear();
				}

				void draw(cv::Mat& map1, cv::Mat& map2, const cv::Scalar& color){
					for (auto point : pt_map1) {
						cv::circle(map1, cv::Point2i(point.y, point.x), 5, color, -1);
					}
					for (auto point : pt_map2) {
						cv::circle(map2, cv::Point2i(point.y, point.x), 5, color, -1);
					}
				}
				
				void getMatchGT(const AASS::graphmatch::GraphLaplacian& graph, const AASS::graphmatch::GraphLaplacian& graph2){
//                     std::deque<MatchVertex> all_matches;
//                     for(auto &match : gt.matches){
//                         MatchVertex matchvertices;
//                         std::cout << "Searching point 1 " << std::endl;
                        for(auto &point_map1 : pt_map1){
                            try{
//                                 std::cout << "Pushed point 1" << std::endl;
                                auto v = graph.getVertex(point_map1);
                                std::cout << "VERTEX FOUND Input " << std::endl;
                                vertex_map1.push_back(v);
                            }
                            catch(std::runtime_error& e){
                                std::cout << "VERTEX NOT FOUND INPUT: " << e.what() << std::endl;
                                
                                cv::Scalar scal(255);
                                cv::Scalar color(150);
                                cv::Mat zone_img = cv::Mat::zeros(1500, 1500, CV_8UC1);
                                graph.drawSpecial(zone_img);
                                cv::circle(zone_img, cv::Point2i(point_map1.y, point_map1.x), 5, color, -1);
                                cv::imshow ("zone test", zone_img);
//                                 cv::waitKey(0);
                            }
                        }
//                         std::cout << "Searching point 2 " << std::endl;
                        for(auto &point_map2 : pt_map2){
                            try{
                                auto v = graph2.getVertex(point_map2);
                                std::cout << "VERTEX FOUND MODEL" << std::endl;
                                vertex_map2.push_back(v);
                            }
                            catch(std::runtime_error& e){
                                std::cout << "VERTEX NOT FOUND MODEL "<< e.what() << std::endl;
                                
                                cv::Scalar scal(255);
                                cv::Scalar color(150);
                                cv::Mat zone_img = cv::Mat::zeros(1500, 1500, CV_8UC1);
                                graph2.drawSpecial(zone_img);
                                cv::circle(zone_img, cv::Point2i(point_map2.y, point_map2.x), 5, color, -1);
                                cv::imshow ("zone test", zone_img);
//                                 cv::waitKey(0);
                            }
                        }
                        
                        
//                         std::cout << " Size " << vertex_map1.size() << std::endl;
//                         std::cout << " Size " << vertex_map2.size() << std::endl;
//                         all_matches.push_back(matchvertices);
//                     }
//                     return all_matches;
                }
                
                
                bool is_tp(const AASS::graphmatch::GraphLaplacian::VertexLaplacian& v1, const AASS::graphmatch::GraphLaplacian::VertexLaplacian& v2) const {
                    
//                     std::cout << " Size " << vertex_map1.size() << std::endl;
//                     std::cout << " Size " << vertex_map2.size() << std::endl;
                    
                    for(auto &v1_t : vertex_map1){
//                         std::cout << "test1 : " << v1_t << " == " << v1 << std::endl;
                        if(v1_t == v1){
                            for(auto &v2_t : vertex_map2){
//                         std::cout << "test1 : " << v2_t << " == " << v2 << std::endl;
                                if(v2_t == v2){
                                    return true;
                                }
                            }
                        }
                    }
                    return false;
                }
                
                bool is_tp(const AASS::graphmatch::evaluation::MatchMaps& matches) const {
                    
//                     std::cout << " Size " << vertex_map1.size() << std::endl;
//                     std::cout << " Size " << vertex_map2.size() << std::endl;
                    
                    for(auto &v1 : matches.vertex_map1){
                        for(auto &v1_t : vertex_map1){
                            if(v1_t == v1){
                                
                                for(auto &v2 : matches.vertex_map2){
                                    for(auto &v2_t : vertex_map2){
                                        if(v2_t == v2){
                                            
                                            return true;
                                            
                                        }
                                    }
                                }
                            }
                        }
                    }
                    return false;
                }



			};

			struct MatchesBetweenMaps {

				std::string map1;
				std::string map2;
				std::vector <MatchMaps> matches;

				MatchesBetweenMaps(const std::string &name, const std::string &name2) : map1(name), map2(name2) {};

				void print() {
					for (int i = 0; i < matches.size(); ++i) {
						std::cout << "# " << i << "th match set\n";
						matches[i].print();
						std::cout << std::endl;
					}
				}

				void export_all(const std::string &file_out) {
					std::string result_file = file_out;
					std::ofstream myfile;
					if (!exists_test3(result_file)) {
						myfile.open(result_file);
						myfile << "# " << map1 << " " << map2 << "\n";
					} else {
						myfile.open(result_file, std::ios::out | std::ios::app);
						myfile << "\n# " << map1 << " " << map2 << "\n";
					}

					if (myfile.is_open()) {
						for (int i = 0; i < matches.size(); ++i) {
							myfile << "# " << i << "match set\n";
							matches[i].export_all(myfile);
							myfile << "\n";
						}
					}
				}

				void draw(cv::Mat& map1, cv::Mat& map2, const cv::Scalar& color) {
					for(auto &match : matches){
						match.draw(map1, map2, color);
					}
				}

				
				void read_file(const std::string& file){

					std::cout << "Read from file: " << file << std::endl;

					std::ifstream infile(file);
					std::string line;

					std::getline(infile, line);
					auto words = getWords(line);
					assert(words[0].compare("#") == 0);
					assert(words.size() == 3);
					this->map1 = words[1];
					this->map2 = words[2];

					std::cout << "Names " << this->map1 << " " << this->map2 << std::endl;

					int count = 0;

					MatchMaps match(this->map1, this->map2);
					while (std::getline(infile, line))
					{

						auto words_tmp = getWords(line);
						if(words_tmp.size() > 0) {
							if (words_tmp[0].compare("#") == 0) {
//								std::cout << "# " << count << std::endl;
								++count;
								if (count == 4) {
									count = 1;
									this->matches.push_back(match);
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
					this->matches.push_back(match);
// 					match.clear();

// 					this->print();
                    
//                     int aaa;
//                     std::cin >> aaa;

				}
				
				void getMatchGT(const AASS::graphmatch::GraphLaplacian& graph, const AASS::graphmatch::GraphLaplacian& graph2){
                    for(auto &&el : matches){
                        el.getMatchGT(graph, graph2);
                    }
                }
                
                
                double tp(const AASS::graphmatch::HypotheseLaplacian& hyp) const {
                    
//                     std::cout << "\n\nBefore tp " << std::endl;
//                     this->print();
//                     int aaaa;
//                     std::cin >> aaaa;
                    
                    double tp = 0;
                    for(auto &match : hyp.getMatches()){
                        auto v1 = match.getFirst();
                        auto v2 = match.getSecond();
                        
//                         std::cout << "Trying the match " << v1 << " " << v2 << std::endl;
                        
                        for(auto &gt_match : matches){
//                             std::cout << "PRINT " << std::endl;
//                             gt_match.print();
                            if(gt_match.is_tp(v1, v2)){
                                tp++;
                            }
                        }
                    }
                    return tp;
                }
                
                double fp(const AASS::graphmatch::HypotheseLaplacian& hyp) const {
                    double fp = 0;
                    for(auto &match : hyp.getMatches()){
                        auto v1 = match.getFirst();
                        auto v2 = match.getSecond();
                        bool is_tp = false;
                        for(auto &gt_match : matches){
                            if(gt_match.is_tp(v1, v2)){
                                is_tp = true;
                            }
                        }
                        if(!is_tp){
                            fp++;
                        }
                    }
                    return fp;
                }
                
                double fn(const AASS::graphmatch::HypotheseLaplacian& hyp) const {
                    double fn = 0;
                    for(auto &match : matches){
                        
                        bool is_tp;
                        for(auto &hyp_match : hyp.getMatches()){
                            auto v1 = hyp_match.getFirst();
                            auto v2 = hyp_match.getSecond();
                            if(match.is_tp(v1, v2)){
                                is_tp = true;
                            }
                        }
                        
                        if(!is_tp){
                            fn++;
                        }
                    }
                    return fn;
                    
                    
                }
                
                
                double tp(const MatchesBetweenMaps& match_model) const {
                    double tp = 0;
                    for(auto &match : matches){
//                         auto v1 = match.getFirst();
//                         auto v2 = match.getSecond();
//                         std::cout << "Trying the match " << v1 << " " << v2 << std::endl;
                        for(auto &gt_match : match_model.matches){
//                             std::cout << "PRINT " << std::endl;
//                             gt_match.print();
                            if(gt_match.is_tp(match)){
                                tp++;
                            }
                        }
                    }
                    return tp;
                }
                
                double fp(const MatchesBetweenMaps& match_model) const {
                    double fp = 0;
                    for(auto &match : matches){
//                         auto v1 = match.getFirst();
//                         auto v2 = match.getSecond();
                        bool is_tp = false;
                        for(auto &gt_match : match_model.matches){
                            if(gt_match.is_tp(match)){
                                is_tp = true;
                            }
                        }
                        if(!is_tp){
                            fp++;
                        }
                    }
                    return fp;
                }
                
                double fn(const MatchesBetweenMaps& match_model) const {
                    double fn = 0;
                    for(auto &match : match_model.matches){
                        
                        bool is_tp;
                        for(auto &hyp_match : matches){
//                             auto v1 = hyp_match.getFirst();
//                             auto v2 = hyp_match.getSecond();
                            if(match.is_tp(hyp_match)){
                                is_tp = true;
                            }
                        }
                        
                        if(!is_tp){
                            fn++;
                        }
                    }
                    return fn;
                    
                    
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
