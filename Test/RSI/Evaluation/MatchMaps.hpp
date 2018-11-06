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

				void print() {
					std::cout << "Point map1 : " << "\n";
					for (auto point : pt_map1) {
						std::cout << point.x << " " << point.y << std::endl;
					}

					std::cout << "Point map2 : " << "\n";
					for (auto point : pt_map2) {
						std::cout << point.x << " " << point.y << std::endl;
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
				}

				void draw(cv::Mat& map1, cv::Mat& map2, const cv::Scalar& color){
					for (auto point : pt_map1) {
						circle(map1, point, 5, color, -1);
					}
					for (auto point : pt_map2) {
						circle(map2, point, 5, color, -1);
					}
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
					for(auto match : matches){
						match.draw(map1, map2, color);
					}
				}


			};

		}
	}
}


#endif