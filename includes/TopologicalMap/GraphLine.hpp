#ifndef GRAPHLIST_LINEFOLLOWER_POINT_MAP
#define GRAPHLIST_LINEFOLLOWER_POINT_MAP

#include <iostream>
#include <stdio.h>
#include <fstream>
#include <assert.h>
#include <stdexcept>

#include "bettergraph/PseudoGraph.hpp"
#include "vodigrex/linefollower/SimpleNodeNamed.hpp"

#include <assert.h>

#include <opencv2/opencv.hpp>

	
namespace AASS{
		
	namespace topologicalmap{
		
		class NodeLine : public vodigrex::SimpleNodeNamed{
		public:
			int force;
			NodeLine() : force(0){};
// 			NodeLine(const NodeLine& sn){
// 				force = 0;
// 				x = sn.getX();
// 				y = sn.getY();
// 			}
		};
		
		/**
		* @brief Graph structure for line extraction
		* 
		* Vertices type is topologicalmap::Intersection_Graph
		* 
		* Edge type is EdgeLineElement
		* 
		*/
		
		class GraphLine : public bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>{
		protected :
			int _marge;
			
		public :
			typedef typename bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Vertex VertexLine;
			typedef typename bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::VertexIterator VertexLineIterator;
			typedef typename bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Edge EdgeLine;
			typedef typename bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::EdgeIterator EdgeLineIterator;
			
			GraphLine() : _marge(10){};
			
			
			//Copy constructor of the other base. This should be transfered to the main class ?
			GraphLine(const bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>& base){
				
				std::vector<VertexLine> vl;
				std::vector<bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Vertex> vl_before;
				std::vector<bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Edge> edge_before;
				
				std::pair<bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::VertexIterator, 				bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::VertexIterator> vp;
				//vertices access all the vertix
				for (vp = boost::vertices(base); vp.first != vp.second; ++vp.first) {
					
					bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Vertex v = *vp.first;
					NodeLine sn = base[v];
					
					VertexLine vline;
					(*this).addVertex(vline, sn);
					vl.push_back(vline);
					vl_before.push_back(v);
				}
				
				for (vp = boost::vertices(base); vp.first != vp.second; ++vp.first) {
					bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Vertex v = *vp.first;
					bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::EdgeIterator out_i, out_end;
					bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Edge e;
					
					for (boost::tie(out_i, out_end) = boost::out_edges(v, base); 
						out_i != out_end; ++out_i) {
						e = *out_i;
					
						bool edge_seen = false;
						for(size_t i = 0 ; i < edge_before.size() ; ++i){
							if(e == edge_before[i]){
								edge_seen = true;
							}
						}
						if(edge_seen == false){
							bettergraph::PseudoGraph<NodeLine, vodigrex::SimpleEdge>::Vertex src = boost::source(e, base), targ = boost::target(e, base);
						
							vodigrex::SimpleEdge edg = base[e];
						
							VertexLine source;
							VertexLine target;
							for(size_t i = 0 ; i < vl_before.size() ; ++i){
								if(vl_before[i] == src){
									source = vl[i];
								}
								if(vl_before[i] == targ){
									target = vl[i];
								}
							}
							
							EdgeLine eline;
							(*this).addEdge(eline, source, target, edg);
							
							edge_before.push_back(e);
						}
						
					}
					
				}
			}
			
			
			virtual void print() const;
			virtual void print(const VertexLine& v) const;
			virtual void draw(cv::Mat& m);
			virtual void draw(const VertexLine& v, cv::Mat& m);
			virtual bool loopDetection(const cv::Point2i& p, const VertexLine& dad, VertexLine& vertex_out);
			
			/** @brief Scale every vertex position by a certain factor
			* 
			* @param[in] scale : scale factor
			*/
			virtual void scale(double scale);
			/// @brief Return the euclidean distance between two vertex
			virtual float distanceSquared(const VertexLine& v, const VertexLine& v2);
// 			virtual void getPoint(const VertexLine& v, cv::Point& p) const;
// 			virtual void setPoint(VertexLine& v, const cv::Point& p){(*this)[v].point = p;}
			/// @brief Return all vertices linked to a vertex
// 			virtual void getAllVertexLinked(const VertexLine& v, std::deque< VertexLine >& all_vertex);
			/// @brief Return all edges and vertices linked to a vertex
// 			virtual void getAllEdgeLinked(const VertexLine& v, std::deque< std::pair< EdgeLine, VertexLine > >& all_edge) const;
			
			/// @brief Fuse v2 and v1 into one vertex with both attributes.
			virtual void fuse(VertexLine& v, VertexLine v_to_remove);
			
			
		};
		
		
		
		inline void GraphLine::print() const
		{
			//first is beginning, second is "past the end"
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
				VertexLine v = *vp.first;
				
				print(v);
				
			}
	// 		std::cout << std::endl;
		}
		inline void GraphLine::draw(const VertexLine& v, cv::Mat& m)
		{
			cv::Scalar color;
			if(m.channels() == 1){
				color = 255;
			}
			else if(m.channels() == 3){
				color[1] = 255;
				color[2] = 255;
				color[3] = 255; 
			}
			cv::Point2i point;
			point.x = (*this)[v].getX();
			point.y = (*this)[v].getY();
			cv::circle(m, point, 5, color);
				
		}

		
		inline void GraphLine::draw(cv::Mat& m)
		{
			
			std::cout << "DRAW" << std::endl;
			
			cv::Scalar color;
			if(m.channels() == 1){
				color = 255;
			}
			else if(m.channels() == 3){
				color[1] = 255;
				color[2] = 255;
				color[3] = 255; 
			}
			//first is beginning, second is "past the end"
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
				
	// 			cv::Mat tmp = cv::Mat::zeros(m.size(), CV_8UC1);
				
	// 			std::cout << "NEW VERTEX" << std::endl;
				
				VertexLine v = *vp.first;
				cv::Point2i point;
				point.x = (*this)[v].getX();
				point.y = (*this)[v].getY();
				cv::circle(m, point, 5, color);
	// 			cv::circle(tmp, (*this)[v].point, 5, color);
	// 
				EdgeIterator out_i, out_end;
				EdgeLine e;
				
				for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)); 
					out_i != out_end; ++out_i) {
	// 				std::cout << "NEW Edge" << std::endl;
					e = *out_i;
					VertexLine src = boost::source(e, (*this)), targ = boost::target(e, (*this));
					cv::Point2i point_src;
					point_src.x = (*this)[src].getX();
					point_src.y = (*this)[src].getY();
					
					cv::Point2i point_targ;
					point_targ.x = (*this)[targ].getX();
					point_targ.y = (*this)[targ].getY();
			
					cv::line(m, point_src, point_targ, color);
	// 				cv::line(tmp, (*this)[src].point, (*this)[targ].point, color);
				
	// 				cv::imshow("the graph being made", m);
	// 				cv::imshow("the graph partial being made", tmp);
	// 				cv::waitKey(0);
				}
				
			}
		}
		
		inline bool GraphLine::loopDetection(const cv::Point2i& p, const VertexLine& dad, VertexLine& vertex_out)
		{
			//Handle loops
			//loop detection
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
	// 		std::cout << "Marge : " << _marge << std::endl; 
			for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
				VertexLine v = *vp.first;
				
	// 			std::cout << "comparing " << p << " to " << (*this)[v].point << std::endl;
				
				if((*this)[v].getX()  < p.x + _marge && (*this)[v].getX() > p.x - _marge){
					if((*this)[v].getY()  < p.y + _marge && (*this)[v].getY() > p.y - _marge){
	// 					std::cout << "same point ! " <<(*this)[v].index << " "<< dad_index << " at " << p.x << " " <<p.y << std::endl;
						//TODO use boost graph index method instead
	// 					std::cout << "SAME so it's a loop" << std::endl;
						vertex_out = v;
						return  false;
									
					}	
				}
			}
	// 		std::cout << " NOT SAME returning dad" << std::endl;
			vertex_out = dad;
			return true;
		}
		
		
		inline void GraphLine::scale(double scale)
		{
				
			//first is beginning, second is "past the end"
			std::pair<VertexIterator, VertexIterator> vp;
			//vertices access all the vertix
			for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
				VertexLine v = *vp.first;
	// 			std::cout << "before " << (*this)[v].point;
				cv::Point2i p;
				p.x = (*this)[v].getX();
				p.y = (*this)[v].getY();
				p = p * scale;
				(*this)[v].setPoint( p.x, p.y);
	// 			std::cout << " after " << (*this)[v].point << std::endl;
			}
		}


		inline void GraphLine::print(const VertexLine& v) const
		{
			std::cout << "Vertex : at position [x: " << (*this)[v].getX() << " y: " << (*this)[v].getX() << "]. Edges : ";

			EdgeIterator out_i, out_end;
			EdgeLine e;
			for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this)); 
				out_i != out_end; ++out_i) {
				e = *out_i;
				VertexLine src = boost::source(e, (*this)), targ = boost::target(e, (*this));
				std::cout << " [x: " << (*this)[src].getX() << " y: " << (*this)[src].getX() << "] -> ";
				std::cout << " [x: " << (*this)[targ].getX() << " y: " << (*this)[targ].getX() << "] || ";
			}
			std::cout << std::endl;
		}
		
		inline float GraphLine::distanceSquared(const VertexLine& v, const VertexLine& v2)
		{
			float dst_x = (*this)[v].getX() - (*this)[v2].getX();
			float dst_y = (*this)[v].getY() - (*this)[v2].getY();
			dst_x = dst_x * dst_x;
			dst_y = dst_y * dst_y;
			return dst_x + dst_y;	
		}

		
		inline void GraphLine::fuse(VertexLine& v, VertexLine v_to_remove)
		{
// 			cv::Point2i other;
// 			getPoint(v_to_remove, other);
// 			cv::Point2i place;
// 			getPoint(v, place);
			
			NodeLine sn = (*this)[v_to_remove];
			cv::Point2i other;
			other.x = sn.getX();
			other.y = sn.getY();
			

			NodeLine sn2 = (*this)[v];
			cv::Point2i place;
			place.x = sn2.getX();
			place.y = sn2.getY();
			
			
			int force = (*this)[v].force;
			int force2 = (*this)[v_to_remove].force;
			
			cv::Point2i new_pos(( (force * place.x) + (force2* other.x)) / (force + force2), ((force * place.y) + (force2* other.y)) / (force + force2));
			
			(*this)[v].setX(new_pos.x);
			(*this)[v].setY(new_pos.y);
			
			//Get all linked vertex of removed vertex :
			std::deque<VertexLine> all_vertex_linked_to_remove;
			getAllVertexLinked(v_to_remove, all_vertex_linked_to_remove);
						
			for(size_t j = 0; j < all_vertex_linked_to_remove.size() ; j++){
				
				//Add edge between old node and all vertice linked to the one to remove.
				if(v != all_vertex_linked_to_remove.at(j)){
					EdgeLine edli;
					addEdge(edli, v, all_vertex_linked_to_remove.at(j));
				}

			}
			
			(*this)[v].force =  force + force2;
			
			removeVertex(v_to_remove);
		}
		


	}
}
#endif