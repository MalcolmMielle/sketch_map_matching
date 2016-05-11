#ifndef PLACEEXTRACTORROOMCORNER_MAP
#define PLACEEXTRACTORROOMCORNER_MAP


#include "PlaceExtractorBase.hpp"
#include "Init.hpp"
#include "MatchingFunctions.hpp"
#include "Keypoints/RoomAndCorner.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"

namespace AASS{

	namespace graphmatch{
		
		/**
		 * @brief Example of a keypoint class.
		 * 
		 * Later, I'd like to have each place extractor having there own keypoint class to play with. This way it will be more controllable.
		 * For now, I still use the base class ad this one is provided as a example for future implementation
		 */
		class AllKeypointRoomCorner : public AllKeypoints{
			
		public :
			AllKeypointRoomCorner(){
				init_all_types_of_keypoints();
			};
			
			virtual void init_all_types_of_keypoints(){	
				//TODO CHANGE FOR MEMORY LEAKS
				clear_all_types_of_keypoints();
								
				Corner* dd = new Corner();
				Keypoint* re = static_cast<Corner*>(dd);
				_all_types_of_keypoints.push_back(re);
				
				Room* r = new Room();;
				_all_types_of_keypoints.push_back(r);

				
			}
			
		};
		
		
		
		
		
		/**
		 * @brief Extract places depending on either :
		 * * Distance to wall
		 * * If there is a wall in between nodes
		 * * Nothing
		 * 
		 * Node are room. Room are at least one junction of voronoi line and some dead end node of the voronoi graph.
		 */
		class PlaceExtractorRoomCorner : public PlaceExtractorBase{
			
		protected:
			
			AASS::vodigrex::ThinkerVoronoi _think;
			cv::Mat _voronoi;
			
			int _distance_fuse;
			int _mode;
			
		public:
			PlaceExtractorRoomCorner() : _distance_fuse(20), _mode(0){};
			virtual ~PlaceExtractorRoomCorner(){}
			
			void reset();
			void inputDistance(const cv::Mat& m){_voronoi = m;}
			void init();
			void extract();

			
			void setMode(int m){_mode = m;}
			void nextMode(){
				
				if(_mode < 2){
					_mode++;
				}
				else{
					_mode = 0;
				}
				std::cout << "the new mode is " << getModeName() << std::endl;
				
			}
			
			int getMode(){return _mode;}
			
			std::string getModeName(){
				if(_mode == 0 ){
					return "Voronoi";
				}else if(_mode == 1 ){
					return "Distanne";
				}else if(_mode == 2 ){
					return "NoObstacle";
				}
				return "error";
			}
			
	// 	private :
			void prunePreviousGraph();
			
			void prunePreviousGraphVoronoi(topologicalmap::GraphLine& graph, topologicalmap::GraphLine::VertexLine& v, topologicalmap::GraphLine::VertexLine& v2);
			
			void prunePreviousGraphDistance(topologicalmap::GraphLine& graph, topologicalmap::GraphLine::VertexLine& v, topologicalmap::GraphLine::VertexLine& v2);
			
			void prunePreviousGraphNoObstacle(topologicalmap::GraphLine& graph, topologicalmap::GraphLine::VertexLine& v, topologicalmap::GraphLine::VertexLine& v2);
			
			void makeGraphV2();
			
			bool noObstacle(const topologicalmap::GraphLine& graph, const topologicalmap::GraphLine::VertexLine& v, const topologicalmap::GraphLine::VertexLine& v_to_remove);
			
			bool isDeadEnd(const topologicalmap::GraphLine::VertexLine v) const;
			bool isCorner(const topologicalmap::GraphLine::VertexLine v) const;

		};
		
		
		inline void PlaceExtractorRoomCorner::extract()
		{
			try{
				std::cout << "PLACE EXTRACTING wiht mode " << _mode << std::endl;
				init();
				std::cout << "Pruning" << std::endl;
				prunePreviousGraph();
				
				
				makeGraphV2();
				
				draw();
				
				std::cout << "Over" << std::endl;
				
			}
			catch(const std::exception &e){
				std::cout << "the unthinkable happened during place extraction : " << e.what() << std::endl;
			}
		}

		inline void PlaceExtractorRoomCorner::init()
		{
	// 		_previous_info_graph.print();
			_map_result = cv::Mat::zeros( _map_in.size(), CV_8UC3 );
			_graph.clear();
			
			std::cout << "VONROI THINKER IN PLACE" << std::endl;
			
// 			cv::imshow("That", _map_in);
// 			cv::waitKey(0);
			_think.voronoi(_map_in);
			
			cv::Mat re = _think.getVoronoi();
			re.copyTo(_voronoi);
			
// 			cv::imshow("VORONOI PLACE", _voronoi);
// 			cv::waitKey(0);
			

		}
		
		inline void PlaceExtractorRoomCorner::reset()
		{
			PlaceExtractorBase::reset();
			_voronoi = cv::Scalar(0);
		}


		
		
		/*
		* SECOND VERSION
		* 
		*/
		//TODO : This function would be better if it would consider all vertex to fuse and then take the median as the "Final vertex" instead of the mean.
		//Reason : the mean can be out of the map.
		
		//Go through all vertices and launch the custom fuse function if two junction are linked
		inline void PlaceExtractorRoomCorner::prunePreviousGraph()
		{
			
			std::cout << "PRUNING THE GRAPH " << std::endl;

	// 		std::cout << "Printing the graph " << std::endl;
	// 		graph.print();
	// 		std::cout << "end" << std::endl;
			
			//Fuse node that are too close using the voronoi distance

			std::pair<topologicalmap::GraphLine::VertexLineIterator, topologicalmap::GraphLine::VertexLineIterator> vp;
			//first is beginning, second is "past the end"
	// 		std::pair<vertex_iter_place, vertex_iter_place> vp;
			
			//ATTENTION : Funny -> if I choose vp from previous graph, but I use graph.addEdge with it, then I modify previous graph and not graph.
			
			for (vp = boost::vertices(_previous_info_graph.getGraph()); vp.first != vp.second; vp.first++) {
				
	// 			std::cout << std::endl << "New Vertice" << std::endl;
				topologicalmap::GraphLine::VertexLine v = *vp.first;
	// 			graph.print(v);
				
				//Should not analyse corners
				if(isCorner(v) == false){
	// 				std::cout << "The vertices is not a corner" << std::endl;
					std::deque<topologicalmap::GraphLine::VertexLine> all_vertex_linked;
					_previous_info_graph.getAllVertexLinked(v, all_vertex_linked);
					
	// 				std::cout << std::endl << "It has " << all_vertex_linked.size() << " vertices linked" << std::endl;
					
					
					for(size_t i = 0 ; i < all_vertex_linked.size(); i++){
						
						//Check that the link is not a corner
						//If not check if need fusion
	// 					std::cout << "in" << std::endl;
	// 					std::cout << "It's a corner " << graph.isCorner(all_vertex_linked.at(i)) << std::endl;
						
						
						if(isCorner(all_vertex_linked.at(i)) == false){
							
							//ATTENTION : IMPORTANT PART IS HERE
							if(_mode == 0){
								prunePreviousGraphVoronoi(_previous_info_graph, v, all_vertex_linked.at(i));
							}if(_mode == 1){
								prunePreviousGraphDistance(_previous_info_graph, v, all_vertex_linked.at(i));
							}if(_mode == 2){
								prunePreviousGraphNoObstacle(_previous_info_graph, v, all_vertex_linked.at(i));
							}
							
						}
						
						//If just, just do nothin
						else{
	// 						std::cout << "Nothing to be done" << std::endl;
						}
					}
					
				}
				
				else{
	// 				std::cout << "It was a corner" << std::endl;
				}
				
			}
			
	// 		std::cout << "OUT" << std::endl;
				

		}
		
		
		
		//Fuse depending on the voronoi distance
		inline void PlaceExtractorRoomCorner::prunePreviousGraphVoronoi(topologicalmap::GraphLine& graph, topologicalmap::GraphLine::VertexLine& v, topologicalmap::GraphLine::VertexLine& v2)
		{
			
			float distance = graph.distanceSquared(v2, v);
			
			vodigrex::SimpleNode sn = graph[v];
			cv::Point2i place;
			place.x = sn.getX();
			place.y = sn.getY();
			

			vodigrex::SimpleNode sn2 = graph[v2];
			cv::Point2i place_2;
			place_2.x = sn2.getX();
			place_2.y = sn2.getY();
			
	// 		std::cout << "Places is : " << place << std::endl;
			//Get the voronoi distance on the point
			float voro_dst = _voronoi.at<float>(place.y, place.x);
			float voro_dst_2 = _voronoi.at<float>(place_2.y, place_2.x);
			
			if(voro_dst < voro_dst_2){
				voro_dst = voro_dst * voro_dst;
			}
			else{
				voro_dst = voro_dst_2 * voro_dst_2;
			}

	// 		voro_dst = voro_dst * voro_dst;
			
			//Compare the two distance. fuse if they are two close
			std::cout << "AT" << std::endl;
			graph.print(v2);
			graph.print(v);
			std::cout << "VORONOI : " << voro_dst << " " << voro_dst_2 << " distance " << distance << std::endl;
			//FUSE if distance condition is good
			if(distance < voro_dst){
				
				graph.fuse(v, v2);
				//Update the vertex
	// 			v = *vp.first;
			}
			
			
		}
		
		
		//Fuse depending on the distance between junctions
		inline void PlaceExtractorRoomCorner::prunePreviousGraphDistance(topologicalmap::GraphLine& graph, topologicalmap::GraphLine::VertexLine& v, topologicalmap::GraphLine::VertexLine& v2)
		{
			float distance = graph.distanceSquared(v2, v);
			
			//Compare the two distance. fuse if they are two close
			
			//FUSE if distance condition is good
			if(distance < _distance_fuse){
				
				graph.fuse(v, v2);
				//Update the vertex
	// 			v = *vp.first;

			}
							
							
						
		}	
		
		
		//Fuse depending on the presence of an obstacle in between 
		inline void PlaceExtractorRoomCorner::prunePreviousGraphNoObstacle(topologicalmap::GraphLine& graph, topologicalmap::GraphLine::VertexLine& v, topologicalmap::GraphLine::VertexLine& v2)
		{
						
			
			//Calculate distance between two juction
			//FUSE if distance condition is good
			if(noObstacle(graph, v, v2) == true){
				
				graph.fuse(v, v2);
				//Update the vertex
	// 			v = *vp.first;

			}
			
			
		}
		
		
		inline bool PlaceExtractorRoomCorner::noObstacle(const AASS::topologicalmap::GraphLine& graph, const AASS::topologicalmap::GraphLine::VertexLine& v, const AASS::topologicalmap::GraphLine::VertexLine& v_to_remove)
		{

			topologicalmap::NodeLine sn = graph[v_to_remove];
			cv::Point2i other;
			other.x = sn.getX();
			other.y = sn.getY();
			

			topologicalmap::NodeLine sn2 = graph[v];
			cv::Point2i place;
			place.x = sn2.getX();
			place.y = sn2.getY();
			
			
			//TODO CHECK IF LINE CROSS ANOTHER LINE
			cv::Mat obstacle = _map_in;
			//Iterate on the line to see if you cross pass with an obstacle
			cv::LineIterator line_it(obstacle, other, place, 8);
			for(int i = 0; i < line_it.count; i++, ++line_it){
				
	// 			std::cout << "point : " <<line_it.pos() << " value : " << (int) _map_in.at<uchar>(line_it.pos()) << " min " << _value_of_white_min << std::endl; ;
				
				if( (int) _map_in.at<uchar>(line_it.pos() ) > _value_of_white_min){
					return false;	
				}
			}
			
			
			return true;

		}

		
		//TODO add edges !!
		inline void PlaceExtractorRoomCorner::makeGraphV2()
		{
						
			cv::Moments m;
			cv::Point2i mass;
			std::vector<cv::Point> contour;
			
			
			std::deque< std::pair < graphmatch::VertexPlace, graphmatch::VertexPlace> > junction_to_link;
			std::deque< std::pair < topologicalmap::GraphLine::VertexLine, graphmatch::VertexPlace> > equivalence_list;
			
			std::pair<topologicalmap::GraphLine::VertexLineIterator, topologicalmap::GraphLine::VertexLineIterator> vp;
			
			//First add all vertex
			
			//first is beginning, second is "past the end"
	// 		std::pair<vertex_iter_place, vertex_iter_place> vp;
			for (vp = boost::vertices(_previous_info_graph.getGraph()); vp.first != vp.second; ++vp.first) {
	// 			Check if it's a dead end.
				topologicalmap::GraphLine::VertexLine v = *vp.first;
				
	// 			If it's not a corner :
				if(isCorner(v) == false){
					
					mass.x = _previous_info_graph[v].getX();
					mass.y = _previous_info_graph[v].getY();
					
					//It's a new junction so we add all the corner that are linked to it
					graphmatch::VertexPlace vplace;
					Place place;
					place.moment = m;
					place.mass_center = mass;
					place.contour = contour;
					_graph.addVertex(vplace, place);
					
					equivalence_list.push_back(std::pair <topologicalmap::GraphLine::VertexLine, graphmatch::VertexPlace >( v, vplace) );
					
					std::deque<topologicalmap::GraphLine::VertexLine> all_vertex_linked;
					_previous_info_graph.getAllVertexLinked(v, all_vertex_linked);
								
					for(size_t j = 0 ; j < all_vertex_linked.size() ; j++){
						if(isCorner(all_vertex_linked.at(j)) == true){
							AASS::vodigrex::SimpleNode inter;
							bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex vert;
							std::pair <AASS::vodigrex::SimpleNode, bettergraph::PseudoGraph<AASS::vodigrex::SimpleNode, AASS::vodigrex::SimpleEdge>::Vertex > pair(_previous_info_graph.getGraph()[all_vertex_linked.at(j)], all_vertex_linked.at(j));
	
							_graph[vplace].landmarks.push_back(pair);
							
						}
						
					}
					
					
				}
				
			}
			
			
			// Second, link them depending on the equivalence list
			
			std::deque< std::pair < topologicalmap::GraphLine::VertexLine, graphmatch::VertexPlace> >::iterator it;
			for(it = equivalence_list.begin() ; it != equivalence_list.end() ; it++){
				
				topologicalmap::GraphLine::VertexLine v = (*it).first;
				std::deque<topologicalmap::GraphLine::VertexLine> all_vertex_linked;
				_previous_info_graph.getAllVertexLinked(v, all_vertex_linked);
				
				for(size_t j = 0 ; j < all_vertex_linked.size() ; j++){
					if(isCorner(all_vertex_linked.at(j)) == false){
						
						//Find the corresponding place
						std::deque< std::pair < topologicalmap::GraphLine::VertexLine, graphmatch::VertexPlace> >::iterator it2 = equivalence_list.begin();
						topologicalmap::GraphLine::VertexLine placelist_to_link = (*it2).first;
						while( placelist_to_link != all_vertex_linked.at(j) ){
							it2++;
							placelist_to_link = (*it2).first;
						}
						graphmatch::VertexPlace place_to_link = (*it2).second;
						
						graphmatch::EdgePlace e;
						_graph.addEdge(e, (*it).second, place_to_link, graphmatch::Gateway());
					}
					
				}
				
			}
			
			AllKeypointRoomCorner all;
			addTypeVertex(_graph, all);
			
			
		}


		
			//TODO : move this to place extractor
		inline bool PlaceExtractorRoomCorner::isCorner(topologicalmap::GraphLine::VertexLine v) const
		{
	// 		std::cout << "REMOVING THIS : " << std::endl;
	// 		print(v);
			//Conisder all cROssing with only one vertice as a dead end.
			if(_previous_info_graph.getNumEdges(v) > 1){
				return false;
			}
			else{
				return true;
			}
		}

		
		//TODO : move this to place extractor
		inline bool PlaceExtractorRoomCorner::isDeadEnd(topologicalmap::GraphLine::VertexLine v) const
		{

	// 		Consider all vertices with at least 2 vertices non dead end connected. It removes the vertices that are the last ones before corner.
			
			int count = 0 ; 

			topologicalmap::GraphLine::EdgeIterator out_i, out_end;
			topologicalmap::GraphLine::Edge e;
			
			//Get out edges
			for (boost::tie(out_i, out_end) = boost::out_edges(v, _previous_info_graph.getGraph()); 
				out_i != out_end; ++out_i) {
				e = *out_i;
				topologicalmap::GraphLine::VertexLine targ = boost::target(e, _previous_info_graph.getGraph());
			
				
				if(_previous_info_graph.getNumEdges(targ) > 1){
					count++;
				}
				
			}
			
	// 		std::cout << "The count is " << count << std::endl;
			if(count < 2){
				return true;
			}
			else{
				return false;
				
			}
		}

		
	}
	
}

#endif
