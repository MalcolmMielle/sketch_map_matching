#ifndef ROOMANDCORNER_MAP
#define ROOMANDCORNER_MAP

#include "MapComparator/GlobalMatch.hpp"
#include "Keypoint.hpp"
#include "MapComparator/Place.hpp"


namespace AASS{
	namespace graphmatch{
		
		struct Room : Keypoint{
			Room() : Keypoint("room"){}
			
			virtual std::string getID() const {return "r";};
			
			virtual bool isOfType(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				std::string str = type;
				std::string str2 = gp[v].getType();
				if(str == str2){
					return true;
				}
				return false;
			};
			
			virtual cv::Scalar getColor(int channel) const{
				
				if(channel == 1){
					return cv::Scalar(160);
				}
				else if(channel == 2){
					cv::Scalar color;
					color[0] = 0;
					color[1] = 0;
					return color;
				}
				else if(channel == 3){
					cv::Scalar color;
					color[0] = 0;
					color[1] = 0;
					color[2] = 255;
					return color;
				}
				else{
					throw std::runtime_error("Weird number of channel");
				}
			}
			
			virtual std::shared_ptr<Keypoint> compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				if(boost::out_degree(v, gp) != 2 || gp[v].landmarks.size() != 1 ){
					std::cout << "CREATING Room" << std::endl;
					auto kp =  makePointer();
					return kp;
				}
				return NULL;
			}
			
			virtual std::shared_ptr<Keypoint> makePointer() const {
				Room* d = new Room();
				Keypoint* re2 = static_cast<Room*>(d);
				std::shared_ptr<Keypoint> re(re2);
				return re;
			}
			
		};
		
		
		struct Corner : Keypoint{
			Corner() : Keypoint("corner"){}
			
			virtual std::string getID() const {return "c";};
			
			virtual bool isOfType(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				std::string str = type;
				std::string str2 = gp[v].getType();
				if(str == str2){
					return true;
				}
				return false;
			};
			
			virtual cv::Scalar getColor(int channel) const{
				
				if(channel == 1){
					return cv::Scalar(120);
				}
				else if(channel == 3){
					cv::Scalar color;
					color[1] = 255;
					color[2] = 0;
					return color;
				}
				else if(channel == 3){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 255;
					color[2] = 0;
					return color;
				}
				else{
					throw std::runtime_error("Weird number of channel");
				}
			}
			
			virtual std::shared_ptr<Keypoint> compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				if(boost::out_degree(v, gp) == 2 && gp[v].landmarks.size() == 1){
					std::cout << "CREATING Corner" << std::endl;
					auto kp =  makePointer();
					return kp;
				}
				return NULL;
			}
			
			virtual std::shared_ptr<Keypoint> makePointer() const {
				Corner* d = new Corner();
				Keypoint* re2 = static_cast<Corner*>(d);
				std::shared_ptr<Keypoint> re(re2);
				return re;
			}
			
		};
	}
}
#endif