#ifndef SKETCHALGO_RSICONVERSION_10032017
#define SKETCHALGO_RSICONVERSION_10032017

#include <RSI/GraphZoneRI.hpp>
#include "GraphPlace.hpp"
#include "LaplacianGraphMatching/GraphLaplacian.hpp"

	
namespace AASS{
		
	namespace graphmatch{
		
		struct ZoneKeypoint : public Keypoint{
			
			AASS::RSI::ZoneRI zone;
			std::string name;
			
			ZoneKeypoint() : Keypoint("zone"){}
			
			void setZone(const AASS::RSI::ZoneRI& zone_in){zone = zone_in;}
			AASS::RSI::ZoneRI& getZone(){return zone;}
			const AASS::RSI::ZoneRI& getZone() const {return zone;}
			
			virtual std::string getID() const {std::cout << "JUST TO MAKE SURE NEVER USE THAT " << std::endl; exit(0); return "z";};
			
			virtual bool isOfType(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				std::string str = type;
				std::string str2 = gp[v].getType();
				if(str == str2){
					return true;
				}
				return false;
			};
			
			virtual bool compareKeypoints(const ZoneKeypoint& k) const {
				auto res = zone.compare(k.zone);
				//Close to 0 is similar, close to 1 is different
				double simi = res.getSimilarity();
				if (simi < 0.5) {
					return true;}
				else{
					return false;
				}
				
			}
			
			virtual bool compareKeypoints(const std::shared_ptr<Keypoint>k) const {
// 				std::cout << "Comparing zone" << name <<" to keypoint" << std::endl;
				
// 				Keypoint* kk = const_cast<Keypoint*>(k);
// 				std::const_pointer_cast<>()
				const std::shared_ptr<ZoneKeypoint> re2 = std::dynamic_pointer_cast<ZoneKeypoint>(k);
				return re2->compareKeypoints(*this);
			}
			
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
			
			virtual std::shared_ptr<Keypoint> compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				throw std::runtime_error("This is no needed, since we don't start with a graphPlace");
			}
			
			virtual std::shared_ptr<Keypoint> makePointer() const {
				ZoneKeypoint* d = new ZoneKeypoint();
				Keypoint* re2 = static_cast<ZoneKeypoint*>(d);
				std::shared_ptr<Keypoint> re(re2);
				return re;
			}
			

			
		};
		
		
		class RSIGraphConverter{

			std::deque < std::pair <AASS::RSI::GraphZoneRI::VertexZone, graphmatch::VertexPlace> > _vertex_equivalent;
			std::deque < std::pair <AASS::RSI::GraphZoneRI::VertexZone, graphmatch::GraphLaplacian::VertexLaplacian> > _vertex_equivalent_laplacian;
			
		public :
			
			RSIGraphConverter(){};

			bool getEquivalentVPlace(const AASS::RSI::GraphZoneRI::VertexZone& in_v, graphmatch::VertexPlace& out){
				for(auto it = _vertex_equivalent.begin(); it != _vertex_equivalent.end() ; ++it){
					if( it->first == in_v){
						out = it->second;
						return true;
					}
				}
				return false;
			}

			bool getEquivalentVZone(const graphmatch::VertexPlace& in_v, AASS::RSI::GraphZoneRI::VertexZone& out){
				for(auto it = _vertex_equivalent.begin(); it != _vertex_equivalent.end() ; ++it){
					if( it->second == in_v){
						out = it->first;
						return true;
					}
				}
				return false;
			}

			bool getEquivalentLaplacianRegion(const AASS::RSI::GraphZoneRI::VertexZone& in_v, graphmatch::GraphLaplacian::VertexLaplacian& out){
				for(auto it = _vertex_equivalent_laplacian.begin(); it != _vertex_equivalent_laplacian.end() ; ++it){
					if( it->first == in_v){
						out = it->second;
						return true;
					}
				}
				return false;
			}

			bool getEquivalentLaplacianZone(const graphmatch::GraphLaplacian::VertexLaplacian& in_v, AASS::RSI::GraphZoneRI::VertexZone& out){
				for(auto it = _vertex_equivalent_laplacian.begin(); it != _vertex_equivalent_laplacian.end() ; ++it){
					if( it->second == in_v){
						out = it->first;
						return true;
					}
				}
				return false;
			}

			///Make sure the graph zone has been updated
			void graphZoneToGraphPlace(const AASS::RSI::GraphZoneRI& graph_zone, AASS::graphmatch::GraphPlace& graph_place){
				std::cout << "START" << std::endl;
// 				try{				
					_vertex_equivalent.clear();
// 					std::deque < std::pair <AASS::RSI::GraphZoneRI::VertexZone, graphmatch::VertexPlace> > dvp;
					std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vptesttt;
					for (vptesttt = boost::vertices(graph_place.getGraph()); vptesttt.first != vptesttt.second; ++vptesttt.first) {
						AASS::graphmatch::VertexPlace v = *vptesttt.first;
						std::cout << "Begining" << std::endl;
						std::static_pointer_cast< ZoneKeypoint > (graph_place[v].getKeypoint())->zone.getMaxMinPCA();		
					}
					
					std::pair<AASS::RSI::GraphZoneRI::VertexIteratorZone, AASS::RSI::GraphZoneRI::VertexIteratorZone> vp;
					//vertices access all the vertices
					for (vp = boost::vertices(graph_zone.getGraph()); vp.first != vp.second; ++vp.first) {
						AASS::RSI::GraphZoneRI::VertexZone v = *vp.first;
						std::cout << "adding vertex " << graph_zone[v].getCentroid() << std::endl;
						graphmatch::VertexPlace vpp;
						cv::Moments mom;
	// 					cv::Point2i center_mass;
	// 					center_mass = graph_zone[v].getCentroid();
	// 					std::cout << "Ading Vertex" << std::endl;
						
						Place p;
						std::cout << "getting countour" << std::endl;
						p.contour = graph_zone[v].getContour();
						p.moment = mom;
						std::cout << "getting centrer" << std::endl;
						p.mass_center = graph_zone[v].getCentroid();
						
						ZoneKeypoint* kp = new ZoneKeypoint();
						std::shared_ptr<ZoneKeypoint> kp_shared(kp);
						std::cout << "set the zone" << std::endl;
						kp_shared->setZone(graph_zone[v]);
						std::cout << "setting keypoint : " << kp << std::endl;
						p.setKeypoint(kp_shared);
						
						//TEST 
						kp_shared->zone.getMaxMinPCA();
						std::static_pointer_cast< ZoneKeypoint >(p.getKeypoint())->zone.getMaxMinPCA();
						
						std::cout << " should be same " << (std::dynamic_pointer_cast< ZoneKeypoint >(p.getKeypoint())).get() << std::endl;
						
						graph_place.addVertex(vpp, p);
						_vertex_equivalent.push_back(std::pair<AASS::RSI::GraphZoneRI::VertexZone, graphmatch::VertexPlace>(v, vpp) );
						
						std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vptestt;
						for (vptestt = boost::vertices(graph_place.getGraph()); vptestt.first != vptestt.second; ++vptestt.first) {
							AASS::graphmatch::VertexPlace v = *vptestt.first;
							auto keypoint = std::dynamic_pointer_cast< ZoneKeypoint >(graph_place[v].getKeypoint());
							std::cout << "TESTING after added vertex for keypoint : " << keypoint << std::endl;
							std::dynamic_pointer_cast< ZoneKeypoint >(graph_place[v].getKeypoint())->zone.getMaxMinPCA();		
						}
	// 					std::cout << "done Adding" << std::endl;
					}
					
					//copy all edges and add room or not
					for (vp = boost::vertices(graph_zone.getGraph()); vp.first != vp.second; ++vp.first) {
						std::cout << "Edge" << std::endl;
						AASS::RSI::GraphZoneRI::VertexZone v = *vp.first;
						std::deque< std::pair< AASS::RSI::GraphZoneRI::EdgeZone, AASS::RSI::GraphZoneRI::VertexZone > > all_edges;
						graph_zone.getAllEdgeLinked(v, all_edges);
						
						graphmatch::VertexPlace dad;
						for(size_t i = 0 ; i < _vertex_equivalent.size() ; i++){
							if(_vertex_equivalent[i].first == v){
								dad = _vertex_equivalent[i].second;
							}
						}
						
						//Add all edges
						for(size_t j = 0 ; j < all_edges.size() ; j++){
							graphmatch::VertexPlace son;
							for(size_t i = 0 ; i < _vertex_equivalent.size() ; i++){
								if(all_edges[j].second == _vertex_equivalent[i].first){
									son = _vertex_equivalent[i].second;
								}
							}
							EdgePlace edge_out;
							graphmatch::Gateway g;
							graph_place.addEdge(edge_out, dad, son, g);
						}
							
					}
					
					std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vptest;
					for (vptest = boost::vertices(graph_place.getGraph()); vptest.first != vptest.second; ++vptest.first) {
						AASS::graphmatch::VertexPlace v = *vptest.first;
						std::cout << "TESTING in functin constructor" << std::endl;
						std::static_pointer_cast< ZoneKeypoint >(graph_place[v].getKeypoint())->zone.getMaxMinPCA();		
					}
					
					
// 				}
// 				catch(const std::exception &e){
// 					std::cout << "problem during graph extraction of place in ListToPLace : " << e.what() << std::endl;
// 				}
				
				
			}


			void graphZonetoGraphLaplacian(const AASS::RSI::GraphZoneRI& graph_zone, AASS::graphmatch::GraphLaplacian& graph_place){

			 	std::cout << "START" << std::endl;
				_vertex_equivalent_laplacian.clear();

				std::pair<AASS::RSI::GraphZoneRI::VertexIteratorZone, AASS::RSI::GraphZoneRI::VertexIteratorZone> vp;
				//vertices access all the vertices
				for (vp = boost::vertices(graph_zone.getGraph()); vp.first != vp.second; ++vp.first) {
					AASS::RSI::GraphZoneRI::VertexZone v = *vp.first;
					std::cout << "adding vertex " << graph_zone[v].getCentroid() << std::endl;
					graphmatch::VertexPlace vpp;
					// 					cv::Point2i center_mass;
					// 					center_mass = graph_zone[v].getCentroid();
					// 					std::cout << "Ading Vertex" << std::endl;

					Region region;
					std::cout << "getting countour" << std::endl;
					region.setContour(graph_zone[v].getContour() ) ;
					std::cout << "getting centrer" << std::endl;
					region.setCenter( graph_zone[v].getCentroid() );
					region.setValue(graph_zone[v].getUniquenessScore() );

					region.zone = graph_zone[v];

					graph_place.addVertex(vpp, region);
					_vertex_equivalent_laplacian.push_back(std::pair<AASS::RSI::GraphZoneRI::VertexZone, graphmatch::GraphLaplacian::VertexLaplacian >(v, vpp) );

					std::cout << "done Adding" << std::endl;
				}

				//copy all edges and add room or not
				for (vp = boost::vertices(graph_zone.getGraph()); vp.first != vp.second; ++vp.first) {
					std::cout << "Edge" << std::endl;
					AASS::RSI::GraphZoneRI::VertexZone v = *vp.first;
					std::deque< std::pair< AASS::RSI::GraphZoneRI::EdgeZone, AASS::RSI::GraphZoneRI::VertexZone > > all_edges;
					graph_zone.getAllEdgeLinked(v, all_edges);

					graphmatch::VertexPlace dad;
					for(size_t i = 0 ; i < _vertex_equivalent_laplacian.size() ; i++){
						if(_vertex_equivalent_laplacian[i].first == v){
							dad = _vertex_equivalent_laplacian[i].second;
						}
					}

					//Add all edges
					for(size_t j = 0 ; j < all_edges.size() ; j++){
						graphmatch::VertexPlace son;
						for(size_t i = 0 ; i < _vertex_equivalent_laplacian.size() ; i++){
							if(all_edges[j].second == _vertex_equivalent_laplacian[i].first){
								son = _vertex_equivalent_laplacian[i].second;
							}
						}
						EdgePlace edge_out;
						graphmatch::EdgeType g;
						graph_place.addEdge(edge_out, dad, son, g);
					}

				}


			 }
		
		};
		
		
		
		
		
		
	}
	
}

#endif