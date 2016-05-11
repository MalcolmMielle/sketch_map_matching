#ifndef PLACEEXTRACTORDOOR_MAP
#define PLACEEXTRACTORLIST2PLACE_MAP


#include "PlaceExtractorBase.hpp"
#include "MatchingFunctions.hpp"

namespace AASS{

	namespace graphmatch{
		
		/**
		 * @brief Keypoint storing class with door junction and dead end
		 */
		class AllKeypointDoorJunctionDeadEnd : public AllKeypoints{
			
		public :
			AllKeypointDoorJunctionDeadEnd(){
				init_all_types_of_keypoints();
			};
			
		public:
			virtual void init_all_types_of_keypoints(){	
				//TODO CHANGE FOR MEMORY LEAKS
				clear_all_types_of_keypoints();
				
// 				Door* door = new Door();
// 				_all_types_of_keypoints.push_back(door);
								
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
		class PlaceExtractorDoors : public PlaceExtractorBase{
			
		protected:
			
			Door _door;
			
		public:
			PlaceExtractorDoors(){};
			virtual ~PlaceExtractorDoors(){}
			
			virtual void extract(){
				_graph.clear();
				AllKeypointDoorJunctionDeadEnd allkey;
				convert(_previous_info_graph, _graph, allkey);
// 				draw();
			};
			
		protected:
			virtual void convert(const AASS::topologicalmap::GraphLine& gl, AASS::graphmatch::GraphPlace& gp, const AllKeypoints& allkey);
			

		};
		
		
		
		inline void PlaceExtractorDoors::convert(const topologicalmap::GraphLine& gl, GraphPlace& gp, const AllKeypoints& allkey)
		{
// 			std::cout << "START" << std::endl;
			try{				
				
				std::deque < std::pair <topologicalmap::GraphLine::VertexLine, graphmatch::VertexPlace> > dvp;
				
				std::pair<topologicalmap::GraphLine::VertexLineIterator, topologicalmap::GraphLine::VertexLineIterator> vp;
				//vertices access all the vertices
				for (vp = boost::vertices(gl.getGraph()); vp.first != vp.second; ++vp.first) {
					topologicalmap::GraphLine::VertexLine v = *vp.first;
					graphmatch::VertexPlace vpp;
					std::vector< cv::Point > contour;
					cv::Moments mom;
					cv::Point center_mass;
					center_mass.x = gl[v].getX();
					center_mass.y = gl[v].getY();
					Place pl;
					pl.moment = mom;
					pl.mass_center = center_mass;
					pl.contour = contour;
// 					std::cout << "Ading Vertex" << std::endl;
					gp.addVertex(vpp, pl);
					dvp.push_back(std::pair<topologicalmap::GraphLine::VertexLine, graphmatch::VertexPlace>(v, vpp) );
// 					std::cout << "done Adding" << std::endl;
				}
				
				//copy all edges and add room or not
				for (vp = boost::vertices(gl.getGraph()); vp.first != vp.second; ++vp.first) {
// 					std::cout << "Edge and rooms" << std::endl;
					topologicalmap::GraphLine::VertexLine v = *vp.first;
					std::deque< std::pair< topologicalmap::GraphLine::Edge, topologicalmap::GraphLine::VertexLine > > all_edges;
					gl.getAllEdgeLinked(v, all_edges);
					
					graphmatch::VertexPlace dad;
					for(size_t i = 0 ; i < dvp.size() ; i++){
						if(dvp[i].first == v){
							dad = dvp[i].second;
						}
					}
					
					//Trying in case it's a door
					std::string type = gl[v].type;
// 					std::cout << "Door type : " << type << std::endl;
					if( type == "Door"){
// 						std::cout << "Adding a door" << std::endl;
						Keypoint* kp = _door.makePointer();
// 							std::cout << "setting the vertice" << std::endl;
						gp[dad].setKeypoint(kp);
					}
					
					
					//Add all edges
					for(size_t j = 0 ; j < all_edges.size() ; j++){
						graphmatch::VertexPlace son;
						for(size_t i = 0 ; i < dvp.size() ; i++){
							if(all_edges[j].second == dvp[i].first){
								son = dvp[i].second;
							}
						}
						EdgePlace edge_out;
						graphmatch::Gateway g;
						gp.addEdge(edge_out, dad, son, g);
					}
						
				}
				
				
				addTypeVertex(gp, allkey);
			}
			catch(const std::exception &e){
				std::cout << "problem during graph extraction of place in ListToPLace : " << e.what() << std::endl;
			}
		}
	}
	
}

#endif