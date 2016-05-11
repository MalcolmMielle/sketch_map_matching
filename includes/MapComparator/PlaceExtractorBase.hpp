#ifndef PLACEEXTRACTOR_MAP
#define PLACEEXTRACTOR_MAP

#include <opencv2/opencv.hpp>
#include "boost/graph/copy.hpp"
#include "GraphPlace.hpp"
#include "TopologicalMap/GraphLine.hpp"
#include "vodigrex/linefollower/SimpleNode.hpp"
#include <stdexcept>

namespace AASS{

	namespace graphmatch{
		
		///@brief Base class of the place extractor
		class PlaceExtractorBase{
			
		protected:
			cv::Mat _map_in; //Map of obstacles
			cv::Mat _map_result; //Map with the places
			graphmatch::GraphPlace _graph;
			topologicalmap::GraphLine _previous_info_graph;
			
			int _scale; //Scale difference between Obstacle compared to Line
			
			int _value_of_white_min;
			int _background_color;
			
			
		public:
			PlaceExtractorBase() : _scale(2), _value_of_white_min(20), _background_color(0){};
			virtual ~PlaceExtractorBase(){}
			
			const graphmatch::GraphPlace& getGraph() const {return _graph;}
			graphmatch::GraphPlace& getGraph() {return _graph;}
			void printGraph(){_graph.print();}
			void drawGraph(cv::Mat& m){_graph.draw(m);}
			virtual void reset();
			void inputMapIn(const cv::Mat& m){
// 				std::cout << "INPUTING MAPS" << std::endl;
// 				cv::cvtColor(m, _map_in, CV_RGB2GRAY);
				m.copyTo(_map_in);
				_map_result = _map_in.clone();
				_map_result = cv::Scalar(0);
			}
			void inputGraph(const topologicalmap::GraphLine& g){
				_previous_info_graph = g;
// 				_previous_info_graph.scale(_scale);
			}
			const cv::Mat& getResult() const {return _map_result;}
			void drawCircle(cv::Mat& in);
			
			///@brief function to call to perform the place extraction 
			virtual void extract() = 0;
			void draw(){_graph.draw(_map_result);}
			


		};
		

		
		inline void PlaceExtractorBase::reset()
		{
			_map_in = cv::Scalar(0);
			_map_result = cv::Scalar(0);
			_graph.clear();
			_previous_info_graph.clear();
		}
	}
}


#endif
