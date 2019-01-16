#ifndef SKETCHALGORIHTM_CREATEHYPFROMSAEED_16012019
#define SKETCHALGORIHTM_CREATEHYPFROMSAEED_16012019

#include "Evaluation.hpp"
#include "RSI/GraphZoneRI.hpp"
#include "RSI/hungarian/hungarian.h"

#include <algorithm>

namespace AASS {
	namespace graphmatch {
		namespace evaluation {

//
//			struct HungMatchSaeed{
////				public:
//				AASS::graphmatch::GraphLaplacian::VertexLaplacian v_input;
//				AASS::graphmatch::GraphLaplacian::VertexLaplacian v_model;
//				int commun_pixels;
//			};


			class CreateHypFromSaeed{

			public:

				CreateHypFromSaeed() = default;


//
//				void createHyp(AASS::graphmatch::GraphLaplacian& graph_input, AASS::graphmatch::GraphLaplacian& graph_model){
//
//
//
//
//				}
//
//
//				AASS::graphmatch::GraphLaplacian::VertexLaplacian bestMatch(AASS::graphmatch::GraphLaplacian::VertexLaplacian input, std::vector<AASS::graphmatch::GraphLaplacian::VertexLaplacian> model_vertices){
//
//				}





				int communPixels(const AASS::graphmatch::Region& first, const AASS::graphmatch::Region& second){

					auto point_zone1 = first.zone.getZone();
					auto point_zone2 = second.zone.getZone();
					int count = 0;
					for(auto el : point_zone1){
						if(std::find(point_zone2.begin(), point_zone2.end(), el) != point_zone2.end()) {
							++count;
						}
					}
					return count;

				}

				std::vector<MatchLaplacian> getMatchesResults(AASS::graphmatch::GraphLaplacian& graph_input, AASS::graphmatch::GraphLaplacian& graph_model){

					std::vector<MatchLaplacian> ret;
					for (auto vp = boost::vertices(graph_input); vp.first != vp.second; ++vp.first) {
						auto v = *vp.first;
						for(auto vp_model = boost::vertices(graph_model); vp_model.first != vp_model.second; ++vp_model.first){
							auto v2 = *vp.first;
							MatchLaplacian match;
							match.setFirst(v);
							match.setSecond(v2);
							match.setCost(communPixels(graph_input[v], graph_model[v2]));
							ret.push_back(match);
						}
					}
					return ret;
				}




				HypotheseLaplacian hungarian(AASS::graphmatch::GraphLaplacian& graph_input, AASS::graphmatch::GraphLaplacian& graph_model){


					auto res = getMatchesResults(graph_input, graph_model);

					int max = -1;
					for(auto el : res){
						if (el.getCost() > max){
							max = el.getCost();
						}
					}
					for(auto el : res){
						el.setCost(- el.getCost() + max);
						assert(el.getCost() >= 0);
						assert(el.getCost() <= max);
					}

//					std::cout << "Wat" << std::endl;
//					for(size_t i = 0 ; i < res.size() ; ++i){
//						std::cout << res[i].getFirst() << " " << res[i].getSecond() << std::endl;
//					}

					std::vector<int> simi;
					for (auto el : res){
						int input = el.getCost();
//						std::cout << " pushing " << input << " because " << it->getCost()*100 << std::endl;
//		assert(input <= 100);
//		assert(input >= 0);
						simi.push_back(input);
					}

					assert((int)simi.size() == graph_input.getNumVertices() * graph_model.getNumVertices());


					/** Lambda **/
					auto array_to_matrix = [] (const std::vector<int>& m, int rows, int cols) -> int** {
						int i,j;
						int** r;
						r = (int**)calloc(rows,sizeof(int*));
						for(i=0;i<rows;i++)
						{
							r[i] = (int*)calloc(cols,sizeof(int));
							for(j=0;j<cols;j++)
								r[i][j] = m[i*cols+j];
						}
						return r;
					};

//	int** array_to_matrix(const std::vector<int>& m, int rows, int cols)




					std::cout << "OUT source on rows and target on cols" << std::endl;
					int** m = array_to_matrix(simi, graph_input.getNumVertices(), graph_model.getNumVertices());

					hungarian_problem_t p;
//	int matrix_size = hungarian_init(&p, m , this->getNumVertices(), laplacian_model.getNumVertices(), HUNGARIAN_MODE_MINIMIZE_COST);
					hungarian_init(&p, m , graph_input.getNumVertices(), graph_model.getNumVertices(), HUNGARIAN_MODE_MINIMIZE_COST);

					/* some output */
					fprintf(stderr, "cost-matrix:");
					hungarian_print_costmatrix(&p);

					std::cout << "Solving" << std::endl;
					/* solve the assignement problem */
					hungarian_solve(&p);
					std::cout << "Solving done" << std::endl;

					/* some output */
					fprintf(stderr, "assignment:");
					hungarian_print_assignment(&p);

					/* some output */
					fprintf(stderr, "cost-matrix:");
					hungarian_print_costmatrix(&p);

//	std::vector< MatchLaplacian > hungarian_matches;
					HypotheseLaplacian hungarian_matches;
					std::vector<int> scores;
					// This depend on which one as more nodes !
					// Goes along the line
					if(graph_input.getNumVertices() <= graph_model.getNumVertices()){

						int i,j;
						// 			fprintf(stderr , "\n");
						for(i=0; i<graph_input.getNumVertices(); i++) {
							// 				fprintf(stderr, " [");
							std::cout << " i " << std::endl;
							for(j=0; j<graph_model.getNumVertices(); j++) {

								std::cout << " w " << std::endl;
								if(p.assignment[i][j] == 1){

									std::cout << "Matching lol2" << i << " with " << j << " cost " << simi.at( ( i*graph_model.getNumVertices() ) + j) << std::endl;

									std::cout << "Matching " << i * graph_model.getNumVertices() << " with " << (i * graph_model.getNumVertices() )  + j << std::endl;
									std::cout << res.at(i * graph_model.getNumVertices()).getFirst() << " " << res.at(( i * graph_model.getNumVertices() ) + j).getSecond() << std::endl;

// 							ZoneCompared m(res.at(i * laplacian_model.getNumVertices()).source, res.at(( i * laplacian_model.getNumVertices() ) + j).target, simi.at( ( i*laplacian_model.getNumVertices() ) + j));

									assert(res.at(( i * graph_model.getNumVertices() ) + j).getFirst() == res.at(i * graph_model.getNumVertices()).getFirst());
									std::cout << "u" << std::endl;
									assert(simi.at( ( i*graph_model.getNumVertices() ) + j) == (int) (res.at(( i * graph_model.getNumVertices() ) + j).getCost() * 100 ));
									std::cout << "u" << std::endl;

									hungarian_matches.push_back(res.at(( i * graph_model.getNumVertices() ) + j));
									std::cout << "u" << std::endl;
									scores.push_back(simi.at( ( i*graph_model.getNumVertices() ) + j));
									std::cout << "u" << std::endl;
								}
							}


							// 				fprintf(stderr, "]\n");
						}
					}
						//Goes down the column
					else{
						assert(hungarian_matches.size() == 0);
						std::cout << "Source more than target" << std::endl;
						int i,j;
						// 			fprintf(stderr , "\n");
						std::cout << "target " << graph_model.getNumVertices() << " source " << graph_input.getNumVertices() << std::endl;
						for(i = 0; i < graph_model.getNumVertices(); i++) {
							// 				fprintf(stderr, " [");
							for(j = 0; j < graph_input.getNumVertices(); j++) {
// 						std::cout << " ass " << p.assignment[j][i] << std::endl;
								if(p.assignment[j][i] == 1){

									std::cout << "Matching lol " << j << " with " << i << " cost " << simi.at( j * graph_model.getNumVertices()) + i << std::endl;

									std::cout << "Matching " <<j * graph_model.getNumVertices()  +i << std::flush << " with " <<  j * graph_model.getNumVertices() + i << std::endl;

									std::cout << res.at(j * graph_model.getNumVertices() + i).getFirst() << " " << res.at(j * graph_model.getNumVertices() + i).getSecond() << std::endl;

// 							ZoneCompared m(res.at((j * laplacian_model.getNumVertices()) + i).source,
// 										   res.at((j * laplacian_model.getNumVertices()) + i).target,
// 										   simi.at( ( i*source.getNumUnique() ) + j));
// 							hungarian_matches.push_back(m);
// 							hungarian_matches.push_back(std::pair<GraphZoneRI::Vertex, GraphZoneRI::Vertex>(
// 								res.at((j * laplacian_model.getNumVertices()) + i).source,
// 								res.at((j * laplacian_model.getNumVertices()) + i).target)
// 							);

// 							std::cout << simi.at( ( i*source.getNumUnique() ) + j) <<"==" << (int) (res.at((j * laplacian_model.getNumVertices()) + i).getSimilarity() * 100 )<< std::endl;

// 							assert(simi.at( ( i*source.getNumUnique() ) + j) == (int) (res.at((j * laplacian_model.getNumVertices()) + i).getSimilarity() * 100 ) );

									hungarian_matches.push_back( res.at((j * graph_model.getNumVertices()) + i) );

									scores.push_back(simi.at( ( i * graph_input.getNumVertices() ) + j));
								}
							}


							// 				fprintf(stderr, "]\n");
						}
					}
// 			fprintf(stderr, "\n");
					std::cout << " hungarian_matches " << std::endl;
//Freeing the memory
					int idx;
					for (idx = 0; idx < graph_input.getNumVertices(); idx += 1) {
						std::cout << "free" << std::endl;
						free(m[idx]);
						std::cout << " afterfree" << std::endl;
					}
					std::cout << "final free" << std::endl;
					free(m);


					std::cout << "outout" << std::endl;
					std::cout << "return " <<hungarian_matches.size() << std::endl;

					for(size_t i = 0 ; i < hungarian_matches.size() ; ++i){
						std::cout << "matching " << i << " : " << hungarian_matches[i].getFirst() << " " << hungarian_matches[i].getSecond() << std::endl;
					}

					std::sort(hungarian_matches.begin(), hungarian_matches.getMatches().end(), [](AASS::graphmatch::MatchLaplacian &match, AASS::graphmatch::MatchLaplacian &match1){
//		return match.getRanking(graph_slam, graph_slam2) > match1.getRanking(graph_slam, graph_slam2);
						return match.getCost() < match1.getCost();
					} );

					return hungarian_matches;
				}


			};



		}
	}
}


#endif