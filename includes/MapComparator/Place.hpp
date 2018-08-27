#ifndef PLACE_MAP_MALCOLM
#define PLACE_MAP_MALCOLM

#include "vodigrex/linefollower/SimpleNode.hpp"

#include "Keypoints/Keypoint.hpp"
#include <opencv2/opencv.hpp>

namespace AASS{
	
	namespace graphmatch{
		/**
		* @brief Vertex stored in GraphPlace
		* 
		* DO NOT FORGET TO INIT THE KEYPOINT !! I found no easy way to use it in the declaration as it is part of a boost Graph
		*/
		class  Place{
			
		protected:
			std::shared_ptr<Keypoint> _kp;
			
			//PROBABILITIES :
			double _variance_edge_number;
			
		public:
			std::deque<graphmatch::Gateway> place;
			std::deque<std::pair <vodigrex::SimpleNode, bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex > > landmarks;
			std::deque<std::pair <vodigrex::SimpleNode, bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex > > junctions;
			std::vector< std::vector< cv::Point > > contour;
			cv::Moments moment;
			cv::Point2f mass_center;
			bool label;
			
// 			Place() : label(false){};
			Place() :  _variance_edge_number(-1), label(false){
				Keypoint* kp = new Keypoint();
				std::shared_ptr<Keypoint> kp_tmp(kp);
				_kp = kp_tmp;
			};
			Place(const Place& cSource) /*: _kp(cSource.makePointer())*/{
				//<thisd is not init yet
// 				delete _kp;
// 				_kp = cSource.makePointer();
				_kp = cSource.getKeypoint();
				place = cSource.place;
				landmarks = cSource.landmarks;
				junctions = cSource.junctions;
				contour = cSource.contour;
				moment = cSource.moment;
				mass_center = cSource.mass_center;
				label = cSource.label;
				
			}
			virtual ~Place(){
// 				delete _kp;
			}
			
			/**
			 * @brief change Keypoint. ATTENTION : argument needs to be created by new. delete is done in destructor so no need to worry.
			 * 
			 * @param[in] k : new Keypoint type pointer. Need to be created from new.
			 */
			void setKeypoint(std::shared_ptr<Keypoint> k){/*deleteKeypoint();*/ _kp = k;}
			
			const std::shared_ptr<Keypoint>& getKeypoint() const {return _kp;}
// 			void deleteKeypoint(){delete _kp;}
			
			void setVariance(const double v){_variance_edge_number = v;}
			double getVariance(){return _variance_edge_number;}
			const double getVariance() const {return _variance_edge_number;}
			
			
			
			///@brief return the one char representing the keypoint
			std::string getID() const {return _kp->getID();}
			std::string getType() const {return _kp->getType();}
			std::shared_ptr<Keypoint> makePointer() const {return _kp->makePointer();}
			///@brief return the color associated with the keypoint. If the keypoint is not init, then it's black.
			cv::Scalar getColor(int channel) const {
				
				if(_kp != NULL){
					return _kp->getColor(channel);
				}
				
				else{
					if(channel == 1){
						return cv::Scalar(0);
					}
					else if(channel == 2){
						cv::Scalar color;
						color[0] = 0 ;
						color[1] = 0 ;
						return color;
					}
					else if(channel == 3){
						cv::Scalar color;
						color[0] = 0 ;
						color[1] = 0 ;
						color[2] = 0 ;
						return color;
					}
					else{
						throw std::runtime_error("Weird number of channel");
					}
				}
			}
			
			Place& operator=(const Place& cSource){
// 				std::cout << "Copying a place" << std::endl;
				if(this != &cSource){
// 					delete _kp;
// 					_kp = cSource.makePointer();
					
					_kp = cSource.getKeypoint();
					place = cSource.place;
					landmarks = cSource.landmarks;
					junctions = cSource.junctions;
					contour = cSource.contour;
					moment = cSource.moment;
					mass_center = cSource.mass_center;
					label = cSource.label;
				}
				return *this;
			}	
			
			bool compare(const Place p) const {
				return _kp->compareKeypoints(p.getKeypoint());
			}
			
		};
		
		
		inline std::istream& operator>>(std::istream& in, Place &p){
	
			std::cout << "READING PLACE" << std::endl;
			in >> p.mass_center.x;
			in >> p.mass_center.y; 
			
			char garbage;
			in >> garbage;
			std::cout << "Garbage " << garbage << std::endl;
			int size = 0;
			in >> size;
			if( garbage == 'G'){
				Gateway g;
				for(int i = 0 ; i < size ; ++i){
					in >> g;
					p.place.push_back(g);
				}
			}
			else{
				throw std::runtime_error("Import fail in Place Gateway");
			}
			
			in >> garbage;
			
			std::cout << "Garbage " << garbage << std::endl;
			if(garbage == 'L'){
				in >> p.label;
			}
			else{
				throw std::runtime_error("Import fail in Place Label");
			}
			
			in >> garbage;
			
			std::cout << "Garbage " << garbage << std::endl;
			if(garbage == 'V'){
				double vaa;
				in >> vaa;
				p.setVariance(vaa);
			}
			else{
				throw std::runtime_error("Import fail in Place Variance");
			}
			
			return in;
		}

		inline std::ostream& operator<<(std::ostream& in, const Place &p){
			
			in << p.mass_center.x << " " << p.mass_center.y << " G " << p.place.size() << " " ;
			
			for(size_t i = 0 ; i < p.place.size() ; ++i){
				in << p.place[i] << " ";
			}
			
			in << " L ";
			
			in << p.label;
			
			in << " V ";
			
			in << p.getVariance();
			
			return in;
			
		}
		
		inline bool comparePlace(const Place& p, const Place& p2){
			return p.compare(p2);
		}
		
		
		
	}
}
#endif