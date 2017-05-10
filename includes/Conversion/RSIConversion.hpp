#ifndef SKETCHALGO_RSICONVERSION_10032017
#define SKETCHALGO_RSICONVERSION_10032017

#include <RSI/GraphZone.hpp>
#include "GraphPlace.hpp"

	
namespace AASS{
		
	namespace graphmatch{
		
		struct ZoneKeypoint : public Keypoint{
			
			AASS::RSI::Zone zone;
			std::string name;
			
			ZoneKeypoint() : Keypoint("zone"){}
			
			void setZone(const AASS::RSI::Zone& zone_in){zone = zone_in;}
			AASS::RSI::Zone& getZone(){return zone;}
			const AASS::RSI::Zone& getZone() const {return zone;}
			
			virtual std::string getID() const {return "z";};
			
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
			
			virtual bool compareKeypoints(const Keypoint* k) const {
// 				std::cout << "Comparing zone" << name <<" to keypoint" << std::endl;
				
// 				Keypoint* kk = const_cast<Keypoint*>(k);
				const ZoneKeypoint* re2 = dynamic_cast<const ZoneKeypoint*>(k);
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
			
			virtual Keypoint* compare(const graphmatch::VertexPlace& v, const GraphType& gp) const{
				throw std::runtime_error("This is no needed, since we don't start with a graphPlace");
			}
			
			virtual Keypoint* makePointer() const {
				ZoneKeypoint* d = new ZoneKeypoint();
				Keypoint* re = static_cast<ZoneKeypoint*>(d);
				return re;
			}
			

			
		};
		
		///Make sure the graph zone has been updated
		void graphZoneToGraphPlace(const AASS::RSI::GraphZone& graph_zone, AASS::graphmatch::GraphPlace& graph_place){
			std::cout << "START" << std::endl;
			try{				
				
				std::deque < std::pair <AASS::RSI::GraphZone::VertexZone, graphmatch::VertexPlace> > dvp;

				
				std::pair<AASS::RSI::GraphZone::VertexIteratorZone, AASS::RSI::GraphZone::VertexIteratorZone> vp;
				//vertices access all the vertices
				for (vp = boost::vertices(graph_zone.getGraph()); vp.first != vp.second; ++vp.first) {
					AASS::RSI::GraphZone::VertexZone v = *vp.first;
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
					std::cout << "set the zone" << std::endl;
					kp->setZone(graph_zone[v]);
					std::cout << "setting keypoint" << std::endl;
					p.setKeypoint(kp);
					
					graph_place.addVertex(vpp, p);
					dvp.push_back(std::pair<AASS::RSI::GraphZone::VertexZone, graphmatch::VertexPlace>(v, vpp) );
// 					std::cout << "done Adding" << std::endl;
				}
				
				//copy all edges and add room or not
				for (vp = boost::vertices(graph_zone.getGraph()); vp.first != vp.second; ++vp.first) {
					std::cout << "Edge" << std::endl;
					AASS::RSI::GraphZone::VertexZone v = *vp.first;
					std::deque< std::pair< AASS::RSI::GraphZone::EdgeZone, AASS::RSI::GraphZone::VertexZone > > all_edges;
					graph_zone.getAllEdgeLinked(v, all_edges);
					
					graphmatch::VertexPlace dad;
					for(size_t i = 0 ; i < dvp.size() ; i++){
						if(dvp[i].first == v){
							dad = dvp[i].second;
						}
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
						graph_place.addEdge(edge_out, dad, son, g);
					}
						
				}
				
				
			}
			catch(const std::exception &e){
				std::cout << "problem during graph extraction of place in ListToPLace : " << e.what() << std::endl;
			}
			
			
		}
		
		
		
		
		
		
	}
	
}

#endif