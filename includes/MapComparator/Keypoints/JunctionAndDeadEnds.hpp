#ifndef JUNCTIONANDDEADENDS_MAP
#define JUNCTIONANDDEADENDS_MAP

#include "GlobalMatch.hpp"
#include "Keypoint.hpp"
#include "Place.hpp"

namespace AASS{
	namespace graphmatch{
		struct Junction : Keypoint{
			Junction() : Keypoint("junction"){}
			
			virtual std::string getID() const {return "j";};
			
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
					return cv::Scalar(200);
				}
				else if(channel == 2){
					cv::Scalar color;
					color[0] = 0;
					color[1] = 255;
					return color;
				}
				else if(channel == 3){
					cv::Scalar color;
					color[0] = 0;
					color[1] = 255;
					color[2] = 0;
					return color;
				}
				else{
					throw std::runtime_error("Weird number of channel");
				}
			}
			
			virtual Keypoint* compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				if(boost::out_degree(v, gp) > 1 ){
					Keypoint* kp =  makePointer();
					return kp;
				}
				return NULL;
			}
			
			virtual Keypoint* makePointer() const {
				Junction* d = new Junction();
				Keypoint* re = static_cast<Junction*>(d);
				return re;
			}
			

			
		};
		
		struct DeadEnd : Keypoint{
			DeadEnd() : Keypoint("deadend"){}
			
			virtual std::string getID() const {return "e";};
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
					return cv::Scalar(70);
				}
				else if(channel == 2){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 0;
					return color;
				}
				else if(channel == 3){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 0;
					color[2] = 255;
					return color;
				}
				else{
					throw std::runtime_error("Weird number of channel");
				}
			}
			
			virtual Keypoint* compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				if(boost::out_degree(v, gp) == 1 ){
					return makePointer();
				}
				return NULL;
			}
			
			virtual Keypoint* makePointer() const {
				DeadEnd* d = new DeadEnd();
				Keypoint* re = static_cast<DeadEnd*>(d);
				return re;
			}
			
		};
		
		struct Door : Keypoint{
			Door() : Keypoint("door"){}
			virtual ~Door(){}
			
			virtual std::string getID() const {return "d";};
			
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
					return cv::Scalar(240);
				}
				else if(channel == 2){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 0;
					return color;
				}
				else if(channel == 3){
					cv::Scalar color;
					color[0] = 255;
					color[1] = 0;
					color[2] = 0;
					return color;
				}
				else{
					throw std::runtime_error("Weird number of channel");
				}
				
			}
			
			
			virtual Keypoint* compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				std::cout << "trying to find a door" << std::endl;
				throw std::runtime_error("No good comparison for a door in place.");
				return NULL;
			}
			
			virtual Keypoint* makePointer() const {
				Door* d = new Door();
				Keypoint* re = static_cast<Door*>(d);
				return re;
			}
			
		};
		
		
	}
}

#endif