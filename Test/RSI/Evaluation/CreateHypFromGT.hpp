#ifndef SKETCHALGORIHTM_CREATEHYPFROMGT_03122018
#define SKETCHALGORIHTM_CREATEHYPFROMGT_03122018

#include "Evaluation.hpp"
#include "RSI/GraphZoneRI.hpp"

namespace AASS{
	namespace graphmatch{
        namespace evaluation{
            
            class MatchVertex{
                
            public: 
                std::deque<AASS::RSI::GraphZoneRI::VertexZoneRI> v_map1;
                std::deque<AASS::RSI::GraphZoneRI::VertexZoneRI> v_map2;
                MatchVertex(){};
                
                bool has(const MatchVertex& matches){
                    for(auto v1 : matches.v_map1){
                        for(auto v2 : matches.v_map2){
                            if(has(v1, v2)){
                                return true;
                            }
                        }
                    }
                    return false;
                }
                
                bool has(const AASS::RSI::GraphZoneRI::VertexZoneRI& first, const AASS::RSI::GraphZoneRI::VertexZoneRI& second){
                    for(auto el : v_map1){
                        if(first == el){
                            for(auto el2 : v_map2){
                                if(second == el2){
                                    return true;
                                }
                            }
                        }
                    }
                    return false;
                }
                
            };
            
        
            class CreateHypFromGT{
                
			protected:

				MatchesBetweenMaps _matches_gt1;
				MatchesBetweenMaps _matches_gt2;
                
            public:
                CreateHypFromGT() : _matches_gt1("", ""), _matches_gt2("", ""){}
                
                void read_file(const std::string& file_input, const std::string& file_model){
                    _matches_gt1.read_file(file_input);
                    _matches_gt2.read_file(file_model);
                }
                
                auto compare(const std::string& gt_input, const std::string& gt_model, AASS::graphmatch::GraphLaplacian& graph_input, AASS::graphmatch::GraphLaplacian& graph_model){
                    
                    read_file(gt_input, gt_model);
                    std::cout << "Finding match gt1 " << std::endl;
                    _matches_gt1.getMatchGT(graph_input, graph_model);
                    std::cout << "Finding match gt2 " << std::endl;
                    _matches_gt2.getMatchGT(graph_input, graph_model);
                    
                    double tp_v = _matches_gt1.tp(_matches_gt2);
                    double fp_v = _matches_gt1.fp(_matches_gt2);
                    double fn_v = _matches_gt1.fn(_matches_gt2);
                    
                    double precision = tp_v / (tp_v + fp_v);
					double recall = tp_v / (tp_v + fn_v);
					double F_measure = (2 * precision * recall) / (precision + recall);

					if(tp_v == 0){F_measure = 0;}

//					double mcc = ( (tp * tn) - (fp * fn) ) / ( std::sqrt( (tp + fn) * (tp + fn) * (tn + fp) * (tn + fn) ));

					return std::make_tuple(tp_v, fp_v, fn_v, precision, recall, F_measure);
                    
                    
                }
                
                int tp(const std::deque<MatchVertex>& matches_gt1, const std::deque<MatchVertex>& matches_gt2){
                    int tp = 0;
                    for(auto match_gt1 : matches_gt1){
                        for(auto match_gt2 : matches_gt2){
                            if(match_gt1.has(match_gt2)){
                                tp++;
                            }
                        }
                    }
                    return tp;
                }
                
                int fp(const std::deque<MatchVertex>& matches_gt1, const std::deque<MatchVertex>& matches_gt2){
                    int fp = 0;
                    for(auto match_gt1 : matches_gt1){
                        bool tp = false;
                        for(auto match_gt2 : matches_gt2){
                            if(match_gt1.has(match_gt2)){
                                tp = true;
                            }
                        }
                        if(!tp){
                            fp++;
                        }
                    }
                    return fp;
                }
                
                int fn(const std::deque<MatchVertex>& matches_gt1, const std::deque<MatchVertex>& matches_gt2){
                    int fn = 0;
                    for(auto match_gt2 : matches_gt2){
                        for(auto match_gt1 : matches_gt1){
                            if(match_gt2.has(match_gt1)){
                                fn++;
                            }
                        }
                    }
                    return fn;
                }
                
                std::deque<MatchVertex> getMatchGT(AASS::RSI::GraphZoneRI& graph, AASS::RSI::GraphZoneRI& graph2, const MatchesBetweenMaps& gt){
                    std::deque<MatchVertex> all_matches;
                    for(auto match : gt.matches){
                        MatchVertex matchvertices;
                        std::cout << "Searching point 1 " << std::endl;
                        for(auto point_map1 : match.pt_map1){
                            try{
                                auto v = getVertex(graph, point_map1);
                                matchvertices.v_map1.push_back(v);
                            }
                            catch(std::runtime_error& e){
                                std::cout << "VERTEX NOT FOUND " << std::endl;
                                
                                cv::Scalar scal(255);
                                cv::Scalar color(150);
                                cv::Mat zone_img = cv::Mat::zeros(1500, 1500, CV_8UC1);
                                graph.draw(zone_img);
                                cv::circle(zone_img, cv::Point2i(point_map1.y, point_map1.x), 5, color, -1);
                                cv::imshow ("zone test", zone_img);
                                cv::waitKey(0);
                            }
                        }
                        std::cout << "Searching point 2 " << std::endl;
                        for(auto point_map2 : match.pt_map2){
                            try{
                                auto v = getVertex(graph2, point_map2);
                                matchvertices.v_map2.push_back(v);
                            }
                            catch(std::runtime_error& e){
                                std::cout << "VERTEX NOT FOUND " << std::endl;
                                
                                cv::Scalar scal(255);
                                cv::Scalar color(150);
                                cv::Mat zone_img = cv::Mat::zeros(1500, 1500, CV_8UC1);
                                graph2.draw(zone_img);
                                cv::circle(zone_img, cv::Point2i(point_map2.y, point_map2.x), 5, color, -1);
                                cv::imshow ("zone test", zone_img);
                                cv::waitKey(0);
                            }
                        }
                        all_matches.push_back(matchvertices);
                    }
                    return all_matches;
                }
                
                auto findRegion(const cv::Point2i& map1_point,  AASS::RSI::GraphZoneRI& graph){
                    for(auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
                        AASS::RSI::GraphZoneRI::VertexZoneRI v = *vp.first;
                        auto zone = (graph)[v].getZone();
                        for(auto point : zone ) {
                            if(point == map1_point){
                                return v;
                            }
                        }
                    }
                }
                
                AASS::RSI::GraphZoneRI::VertexZoneRI getVertex(AASS::RSI::GraphZoneRI& graph, const cv::Point2i& map_point){
                    
                    
                    AASS::RSI::GraphZoneRI::VertexZoneRI v_closest_if_not_found;
                    double smallest_dist = -1;
                    
                    for(auto vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
                        AASS::RSI::GraphZoneRI::VertexZoneRI v = *vp.first;
                        auto zone = (graph)[v].getZone();
                        
//                         cv::Scalar scal(255);
//                         cv::Scalar color(150);
//                         cv::Mat zone_img = cv::Mat::zeros(500, 500, CV_8UC1);
//                         (graph)[v].drawZone(zone_img, scal);
//                         cv::circle(zone_img, cv::Point2i(map_point.y, map_point.x), 5, color, -1);
//                         cv::imshow ("zone test", zone_img);
//                         cv::waitKey(0);
                        
                        
//                         auto region = graph[v];
//                         auto zone = region.getZone();
                        for(auto point : zone ){
                            
                            double dist = cv::norm(map_point - point);
                            if(map_point == point){
                                std::cout << "FOUND" << std::endl;
                                return v;
                            }
                            else{
                                if(dist <= smallest_dist || smallest_dist == -1){
                                    smallest_dist = dist;
                                    v_closest_if_not_found = v;
                                }
                            }
                        }
                    }
                    
                    std::cout << map_point << " at point " << std::endl;
                    if(smallest_dist <= 10){
                        return v_closest_if_not_found;
                    }
                    throw std::runtime_error("Zone not found");
// 					std::cout << "Not found" << std::endl;
				}
                
                
            };
            
        }
    }
}

#endif
