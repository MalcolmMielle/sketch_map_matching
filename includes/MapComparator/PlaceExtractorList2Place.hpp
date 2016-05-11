#ifndef PLACEEXTRACTORLIST2PLACE_MAP
#define PLACEEXTRACTORLIST2PLACE_MAP


#include "PlaceExtractorBase.hpp"
#include "MatchingFunctions.hpp"

namespace AASS{

	namespace graphmatch{
		
		/**
		 * @brief Keypoint storing class with junction and dead end
		 */
		class AllKeypointJunctionDeadEnd : public AllKeypoints{
			
		public :
			AllKeypointJunctionDeadEnd(){
				init_all_types_of_keypoints();
			};
			
		public:
			virtual void init_all_types_of_keypoints(){	
				//TODO CHANGE FOR MEMORY LEAKS
				clear_all_types_of_keypoints();
								
				Junction* dd = new Junction();
				Keypoint* re = static_cast<Junction*>(dd);
				_all_types_of_keypoints.push_back(re);
				
				DeadEnd* rr = new DeadEnd();
				Keypoint* rerr = static_cast<DeadEnd*>(rr);
				_all_types_of_keypoints.push_back(rerr);
				
			}
			
		};
		
		/**
		 * @brief Directly translate a graph list to a place one
		 * 
		 */
		class PlaceExtractorList2Place : public PlaceExtractorBase{
			
		protected:
			
		public:
			PlaceExtractorList2Place(){};
			virtual ~PlaceExtractorList2Place(){/*std::cout << "Rmeoving place xtractor list2place" << std::endl;*/}
			
			virtual void extract(){
				_graph.clear();
// 				std::cout << "Size graph " << _graph.getNumVertices() << std::endl;
				AllKeypointJunctionDeadEnd all;
				fromList2Place(_previous_info_graph, _graph, all);
// 				std::cout << "drawing" << std::endl;
				draw();
// 				cv::imshow("Result before", _map_result);
// 				cv::waitKey(0);
			};
			

		};
	}
	
}

#endif
