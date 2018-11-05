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

#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/HypotheseLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"

#include "MatchMaps.hpp"

namespace AASS{
	namespace graphmatch{
		namespace evaluation{

			class Evaluation{

			protected:

				MatchesBetweenMaps _matches;

			public:
				Evaluation() : _matches("", ""){}

				void read_file(const std::string& file){

					std::cout << "Read from file" << std::endl;

					std::ifstream infile(file);
					std::string line;

					std::getline(infile, line);
					auto words = getWords(line);
					assert(words[0].compare("#") == 0);
					assert(words.size() == 3);
					_matches.map1 = words[1];
					_matches.map2 = words[2];

//					std::cout << "Names " << _matches.map1 << " " << _matches.map2 << std::endl;

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

				double evaluate(const AASS::graphmatch::HypotheseLaplacian& hyp, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					double count = 0;
					for(auto match : hyp.getMatches()){
						if(is_correct(match, gl, gl_model) ){
							++count;
						}
					}
					return count / (double) hyp.size();

				}

				bool is_correct(const AASS::graphmatch::MatchLaplacian& match, const AASS::graphmatch::GraphLaplacian& gl, const AASS::graphmatch::GraphLaplacian& gl_model){

					auto v1 = match.getFirst();
					auto v2 = match.getSecond();
					auto region1 = gl[v1];
					auto region_model = gl[v2];

					auto zone = region1.zone.getZone();

					for(auto point : zone ){

						for(auto mapmatches : _matches.matches){
							for(auto point_map1 : mapmatches.pt_map1){
								if(point_map1 == point){
									auto zone_model = region_model.zone.getZone();
									for(auto point_model : zone_model ){
										for(auto point_map2 : mapmatches.pt_map2) {
											if (point_map2 == point_model) {
												std::cout << "Found with " << point << " and " << point_model << std::endl;
												return true;
											}
										}
									}

								}
							}
						}

					}
					std::cout << "Not found" << std::endl;

					return false;

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