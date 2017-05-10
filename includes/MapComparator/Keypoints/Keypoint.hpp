#ifndef KEYPOINT_MAP
#define KEYPOINT_MAP

#include <stdexcept>
#include <opencv2/opencv.hpp>
#include "MapComparator/GlobalMatch.hpp"

//INTERFACE 

namespace AASS{
	namespace graphmatch{
		
		class Place;
		/**
		 * @brief General type of Vertex to make the matching algorithm modulable. don't know if should put it here or topologicalmap
		 * 
		 * Making it full virtual was a pain (because then I need to declare instance that are not temporary and then I can't copy because it's pointer that is stored in Place.
		 * 
		 * So I make it an "unusable class", as soon as you wish to use it, it will crash if it's a Keypoint class. It won't if it's something else.)
		 */		
		struct Keypoint{
			
			std::string type;
			
			Keypoint(const std::string& typet) : type(typet){}
			Keypoint() : type("notype"){}
			
			virtual ~Keypoint(){}
			
			virtual std::string getType() const {return type;}
			
			///@brief return a one character representing the type
			virtual std::string getID() const {
				throw std::runtime_error("Trying to use a Keypoint base class to get ID."); 
				return "";	
			}
			
			///@brief return true if the GraphPlace::VertexPlace is of *this type. The simple version just compare IDs but I should do better
			virtual bool isOfType(const VertexPlace& v){
				throw std::runtime_error("Trying to use a Keypoint base class to get type."); 
				return false;
			}
			
			/**
			 * @brief return a pointer to an element on this class if the vertex is determine to be of this class. Used for creating new instances of the keypoint if the vertex is of the keypoint type. This method is very important
			 * @param[in] v : vertex in graphmatch
			 * @param[in] gp : graph to which the vertex belong
			 */
			virtual Keypoint* compare(const VertexPlace& v, const GraphType& gp) const{
				throw std::runtime_error("Trying to use a Keypoint base class to get type.");
				return NULL;
			}
			
			///@brief return true if k and this are of the "same".
			virtual bool compareKeypoints( const Keypoint* k) const{
				throw std::runtime_error("Trying to use a Keypoint base class to get a comparison to another keypoints.");
				return NULL;
			}
			
			virtual Keypoint* makePointer() const {
				std::cout << "MAKING A BASE POINTER NOT GOOOOD" << std::endl;
				Keypoint* d = new Keypoint();
				return d;
			}
			
			virtual cv::Scalar getColor(int channel) const{
				std::cout << "base function NOT GOOD" << std::endl;
				if(channel == 1){
					return cv::Scalar(255);
				}
				else if(channel == 2){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 255;
					return color;
				}
				else if(channel == 3){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 255;
					color[2] = 255;
					return color;
				}
				else{
					throw std::runtime_error("Weird number of channel");
				}
			}
			
		};
		
		
	}
}

#endif