#ifndef INITINITINIT_MAP
#define INITINITINIT_MAP

#include "Keypoints/Keypoint.hpp"
#include "Keypoints/JunctionAndDeadEnds.hpp"
#include "Keypoints/RoomAndCorner.hpp"


//TODO MAKE THIS ACLASS TO AVOID MEMORY LEAKS

namespace AASS{
	namespace graphmatch{
		/**
		 * @brief Class container of all type of nodes
		 * 
		 * This class must be used to set the type of vertex in graph place. For this purpose, _all_types_of_keypoints need to include different type of vertex with a compare method to determine if the input vertex is of the same type. It is on the user to make sure that no Vertex is left without a type !!
		 * 
		 * Later should be made virtual and every place extractor should use a custom AllKeypoint class. See example class in MatchingFucntion and PlaxeExtractorRoomCorner
		 * 
		 */
		class AllKeypoints{
			
		protected:
			std::vector <Keypoint*> _all_types_of_keypoints;
			
		public:
			AllKeypoints(){
// 				init_all_types_of_keypoints();
			}
			virtual ~AllKeypoints(){
				clear_all_types_of_keypoints();
			}
			
			std::vector <Keypoint*>& getAll(){return _all_types_of_keypoints;}
			Keypoint* operator[](const int i){return _all_types_of_keypoints[i];}
			const std::vector <Keypoint*>& getAll() const {return _all_types_of_keypoints;}
			const Keypoint* operator[](const int i) const {return _all_types_of_keypoints[i];}
			
			
			
// 		protected:
			///@brief init all Keypoints type in the deque
			virtual void init_all_types_of_keypoints() = 0 ;
			/*{	
				clear_all_types_of_keypoints();
								
// 				Corner* dd = new Corner();
// 				Keypoint* re = static_cast<Corner*>(dd);
// 				_all_types_of_keypoints.push_back(re);
// 				
// 				Room* r = new Room();;
// 				_all_types_of_keypoints.push_back(r);
				
				Junction* dd = new Junction();
				Keypoint* re = static_cast<Junction*>(dd);
				_all_types_of_keypoints.push_back(re);
				
				DeadEnd* r = new DeadEnd();
				_all_types_of_keypoints.push_back(r);
				
				
			}*/
			
			///@brief clear all the pointer contained by the class.
			virtual void clear_all_types_of_keypoints(){
				for(typename std::vector <Keypoint*>::iterator it = _all_types_of_keypoints.begin() ; it!= _all_types_of_keypoints.end() ;){
					delete(*it);
					_all_types_of_keypoints.erase(it);
				}
			}
			
		};
	}
}

#endif