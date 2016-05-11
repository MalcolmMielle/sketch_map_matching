#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "Util.hpp"
#include "GraphMatcherClusterFiltered.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "SketchMap.hpp"
#include "Thinker_Voronoi.hpp"
#include "PlaceExtractorList2Place.hpp"

#include "ProbabilisticEditDistanceEdge.hpp"
#include "JunctionAndDeadEnds.hpp"

#include <ctime>


int correctMatch(const std::deque < AASS::graphmatch::Match>& cm, const AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::GraphPlace& gp_model){
	int count = 0 ;
	for(size_t i = 0 ; i < cm.size() ; i++){
		if(gp[cm[i].getFirst()].mass_center.x == gp[cm[i].getSecond()].mass_center.x && gp[cm[i].getFirst()].mass_center.y == gp[cm[i].getSecond()].mass_center.y){
			count ++;
		}
	}
	return count;
}


BOOST_AUTO_TEST_CASE(trying)
{
	
	srand (time(NULL));
	
// 	int nb_diff = 1;
// 	std::cout << "How much difference between graphs" << std::endl;
// 	std::cin >> nb_diff;
	int count = 0;
	int bad = 0 ;
	int vertex_matched_good = 0;
	int bad_ped = 0 ;
	int vertex_matched_good_ped = 0;
	std::deque < int > dresults ;
	std::deque < int > dmean ;
	std::deque < double > time ;
	std::deque < int > dresults_ped ;
	std::deque < int > dmean_ped ;
	std::deque < double > time_ped ;
	for(size_t num_vertex = 10 ; num_vertex < 60 ; num_vertex = num_vertex +10){
	
		for(size_t error = 0 ; error < 6 ; error++){
			
			dresults.push_back(num_vertex);
			dresults.push_back(error);
			dresults_ped.push_back(num_vertex);
			dresults_ped.push_back(error);
			int mean = 0 ;
			int mean_ped = 0 ;
			double timi;
			double timi_ped;
			for(size_t i = 1 ; i < 2 ; i++){
				
				std::cout << std::endl << "TEST : " << i << " with " << error << " differences and " << num_vertex << " vertices." <<  std::endl << std::endl;;
				AASS::graphmatch::GraphPlace gp;
				AASS::graphmatch::GraphPlace gp_copy;
				std::deque < AASS::graphmatch::VertexPlace > dplace;
				std::deque < AASS::graphmatch::VertexPlace > dplace_copy;
				std::pair< int, int > size_image(400, 400);
				
				int cost = 0 ;
// 				int cost = AASS::graphmatch::addRandomVertex(gp, size_image, dplace);
// 				std::cout << "added vertex cost : " << cost << std::endl;
				
// 				std::cout << "Create graph" << std::endl;
				AASS::graphmatch::AllKeypointJunctionDeadEnd allkey;
				AASS::graphmatch::createGraph(gp, gp_copy, num_vertex, num_vertex + 5, size_image, dplace, dplace_copy);
				
				for(size_t ii = 0 ; ii < error ; ii ++){
					cost = cost + AASS::graphmatch::addRandomVertex(gp, size_image, dplace);
					cost = cost + AASS::graphmatch::addRandomEdge(gp, dplace);
// 					std::cout << "added edge cost : " << cost << std::endl;
					
		// 			mat = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
		// 			gp.drawSpecial(mat);
		// 			cv::imshow("graph", mat);
		// 			mat_model = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
		// 			gp_copy.drawSpecial(mat_model);
		// 			cv::imshow("graph model", mat_model);
		// 			cv::waitKey(0);
				}
				
// 				if(error > 0){
// 					cv::Mat mat = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
// 					gp.drawSpecial(mat);
// 					cv::imshow("graph", mat);
// 					cv::Mat mat_model = cv::Mat::zeros(cv::Size(400, 400), CV_8UC3);;
// 					gp_copy.drawSpecial(mat_model);
// 					cv::imshow("graph model", mat_model);
// 					cv::waitKey(0);
// 				}

				addTypeVertex(gp, allkey);
				addTypeVertex(gp_copy, allkey);
				
				AASS::graphmatch::GraphMatcherClusterFiltered ccf;
				AASS::graphmatch::GraphMatcherNeighbor cc;				
				AASS::probabilisticmatching::ProbabilisticEditDistanceEdge ped;
				
// 				std::cout << "doing planar" << std::endl;
				std::clock_t a = std::clock();
				bool res = cc.planarEditDistanceAlgorithm(gp_copy, gp);
				std::clock_t b = std::clock();
				
// 				std::cout << "doing algo4" << std::endl;
				std::clock_t a_ped = std::clock();
				bool res_ped = ped.algo4(gp_copy, gp);
				std::clock_t b_ped = std::clock();
// 				std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< " or in minutes " << ( ((float)(b - a))/CLOCKS_PER_SEC) / 60 << std::endl;
				
		// 		a = std::clock();
		// 		bool res = ccf.planarEditDistanceAlgorithm();
		// 		b = std::clock();
		// 		std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< " or in minutes " << ( ((float)(b - a))/CLOCKS_PER_SEC) / 60 << std::endl;
				
				
				
				if(res == true){
		// 			std::deque<	
		// 					AASS::graphmatch::Hypothese
		// 				> hypothesis_final = ccf.getResult();
		// 			ccf.sort(hypothesis_final);
		// 			ccf.drawHypo(gp_copy, gp, hypothesis_final[0].getMatches(), "ALL FINAL");
		// 			
		// 			BOOST_CHECK_EQUAL(hypothesis_final[0].getDist(), cost);
					
					std::deque<	
							AASS::graphmatch::Hypothese
						> hypothesis_final_cc = cc.getResult();
					cc.sort(hypothesis_final_cc);
// 					cv::Mat mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
// 					cc.drawHypo(gp_copy, gp, mat_in, mat_in, hypothesis_final_cc[0].getMatches(), "ALL FINAL CC");
// 					cv::waitKey(0);
		// 			std::cout << "at the end matched " << correctMatch(hypothesis_final_cc[0].getMatches(), gp, gp_copy) << std::endl;
					
					vertex_matched_good = vertex_matched_good + correctMatch(hypothesis_final_cc[0].getMatches(), gp, gp_copy);
					
					BOOST_CHECK_EQUAL(hypothesis_final_cc[0].getDist(), cost);
					if(hypothesis_final_cc[0].getDist() != cost){
						bad ++ ;
					}
					timi = timi + ( ((float)(b - a))/CLOCKS_PER_SEC);
				}
				else{
// 					std::cout << "No result found" << std::endl;
				}
				
				if(res_ped == true){
		// 			std::deque<	
		// 					AASS::graphmatch::Hypothese
		// 				> hypothesis_final = ccf.getResult();
		// 			ccf.sort(hypothesis_final);
		// 			ccf.drawHypo(gp_copy, gp, hypothesis_final[0].getMatches(), "ALL FINAL");
		// 			
		// 			BOOST_CHECK_EQUAL(hypothesis_final[0].getDist(), cost);
					
					std::deque<	
							AASS::graphmatch::Hypothese
						> hypothesis_final_ped = ped.getResult();
					cc.sort(hypothesis_final_ped);
// 					cv::Mat mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
// 					cc.drawHypo(gp_copy, gp, mat_in, mat_in, hypothesis_final_ped[0].getMatches(), "ALL FINAL CC PED");
// 					cv::waitKey(0);
		// 			std::cout << "at the end matched " << correctMatch(hypothesis_final_cc[0].getMatches(), gp, gp_copy) << std::endl;
					
					vertex_matched_good_ped = vertex_matched_good_ped + correctMatch(hypothesis_final_ped[0].getMatches(), gp, gp_copy);
					
					BOOST_CHECK_EQUAL(hypothesis_final_ped[0].getDist(), cost);
					if(hypothesis_final_ped[0].getDist() != cost){
						bad_ped ++ ;
					}
// 					std::cout << "Cloclks " << CLOCKS_PER_SEC << std::endl;
					timi_ped = timi_ped + ( ((float)(b_ped - a_ped))/CLOCKS_PER_SEC);
				}
				else{
// 					std::cout << "No result found" << std::endl;
				}
				
		// 		cv::waitKey(0);
// 			std::cout << "Adding the count" << std::endl;
			count++;	
			mean = mean + cost;
			mean_ped = mean_ped + cost;
			}
			
			vertex_matched_good = vertex_matched_good / num_vertex;
			vertex_matched_good_ped = vertex_matched_good_ped / num_vertex;
			
			dresults.push_back(vertex_matched_good);
			dresults_ped.push_back(vertex_matched_good_ped);
			
			mean = mean / 100;
			mean_ped = mean_ped / 100;
			timi = timi / 100;
			timi_ped = timi_ped / 100;
			
			dmean.push_back(mean);
			dmean_ped.push_back(mean_ped);
			time.push_back(timi);
			time_ped.push_back(timi_ped);
			
// 			std::cout << "In the mean of good match is : " << vertex_matched_good << std::endl;
			
// 			std::cout << "number of bad : " << bad << " over : " << count <<std::endl;

			
		}
	}
	
	for(size_t i = 0 ; i < dresults.size() ; i = i+3){
		std::cout << "vertex : " << dresults[i] << " error " << dresults[i+1] << " percentage of good match " << dresults[i+2] << " mean " << dmean[i/3] << " time " << time[i/3] << std::endl;
		std::cout << "ped vertex : " << dresults_ped[i] << " error " << dresults_ped[i+1] << " percentage of good match " << dresults_ped[i+2] << " mean " << dmean_ped[i/3] << " time " << time_ped[i/3] << std::endl;
	}
	
}