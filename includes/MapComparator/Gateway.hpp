#ifndef GATEWAY_MAP
#define GATEWAY_MAP

#include <opencv2/opencv.hpp>
namespace AASS{
	
	namespace graphmatch{
		
		///@brief represent a door 
		class Gateway{
			
		protected:
			std::pair< cv::Point2i, cv::Point2i > _gateway;
		public:
			bool label;
			
			Gateway() : label(true){
				_gateway.first = cv::Point2i(0 ,0);
				_gateway.second = cv::Point2i(0 ,0);
			};
			
			Gateway(const cv::Point2i& p, const cv::Point2i& p2) : label(true){
				_gateway.first = p;
				_gateway.second = p2;
			}
			
			const std::pair< cv::Point2i, cv::Point2i >& getGateway() const {return _gateway;}
			
			void setFirst(const cv::Point2i& p){_gateway.first = p;}
			void setSecond(const cv::Point2i& p){_gateway.second = p;}
			void setGateway(const std::pair< cv::Point2i, cv::Point2i >& pair){_gateway = pair;}
			void setGateway(const cv::Point2i& p, const cv::Point2i& pp){
				_gateway.first = p;
				_gateway.second = pp;
			}
			
			const cv::Point2i& getFirst() const {return _gateway.first;}
			const cv::Point2i& getSecond() const {return _gateway.second;}
			
			void print() const {
				std::cout << "First point : " << _gateway.first << " second point " << _gateway.second;
			}
			
			
			//TODO epsilon detection
			virtual bool isCrossingLine(const cv::Point& p0, const cv::Point& p1){
				
				float s10_x, s10_y, s32_x, s32_y;
				s10_x = p1.x - p0.x;
				s10_y = p1.y - p0.y;
				s32_x = _gateway.second.x - _gateway.first.x;
				s32_y = _gateway.second.y - _gateway.first.y;
			
				std::cout << s10_x <<" " <<s10_y <<" " << s32_x <<" "<< s32_y <<" "<< std::endl;
				
				float denom = s10_x * s32_y - s32_x * s10_y;
				
				if(denom == 0){
					std::cout << "Denom is zero" << std::endl;
					return false;
						//Collinear
				}
				
				bool denomPositive = denom > 0 ;
				
				float s_numer, t_numer, s02_x, s02_y;
				s02_x = p0.x - _gateway.first.x;
				s02_y = p0.y - _gateway.first.y;
				
				s_numer = s10_x * s02_y - s10_y * s02_x;
				
				if((s_numer < 0) == denomPositive){
					std::cout << "first cond" << std::endl;
					return false;
				}
				
				t_numer = s32_x * s02_y - s32_y * s02_x;
				
				if((t_numer < 0) == denomPositive){
					std::cout << "second is zero" << std::endl;
					return false;
				}
			
				if ((s_numer > denom) == denomPositive || ((t_numer > denom) == denomPositive))
				{
					// No cllision
					std::cout << "third is zero" << std::endl;
					return false;
				}
			
				return true; // No collision
			}
			
		};
		
		
		inline std::ostream& operator<<(std::ostream& in, const Gateway &p){
			
			return in << p.getFirst().x << " " << p.getFirst().y << " " \
			<< p.getSecond().x << " " << p.getSecond().y << " " << p.label ;
			
		}
		
		inline std::istream& operator>>(std::istream& in, Gateway &p){
			cv::Point2i poi;
			
			in >> poi.x;
			in >> poi.y;
			p.setFirst(poi);
			
			in >> poi.x;
			in >> poi.y;
			p.setSecond(poi);
			
			in >> p.label;
			
			return in;
		}
		
	}
}


#endif
