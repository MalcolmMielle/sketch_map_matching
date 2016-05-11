#ifndef STORAGEFILTEREDCOMPARATOR_MAP
#define STORAGEFILTEREDCOMPARATOR_MAP

#include "GraphPlace.hpp"
/*
 * TODO : I need a deep copy of the graph or the function to reduce it doesn't work at the remove part.
 * Thus for now it's impossible to use the attributes
 * 
 */


namespace AASS{
		
	namespace graphmatch{
		
		/**
		 * @brief Storing class for the vertex and edge to remove or added of a graphmatch::GraphPlace
		 * 
		 * This class store all the vertex and edges modification applied to a graphmatch::GraphPlace while being able to reset the topologicalmap::Graph Place.
		 * 
		 * The graphmatch::GraphPlace is temporilly modified by adding edges but can be return to its original state. 
		 */
		
		class StorageFilter{
			
		protected :
			
			boost::keep_all _keep_all;
			std::deque<graphmatch::VertexPlace> _removed_set_vertex;
			std::deque<graphmatch::EdgePlace> _removed_set_edge;
			std::deque<graphmatch::EdgePlace> _added_edges;
			
		public :
			
			StorageFilter(){};
			
			virtual void addToRemove(const graphmatch::VertexPlace& v){_removed_set_vertex.push_back(v);}
			virtual void addToRemove(const graphmatch::EdgePlace& e){_removed_set_edge.push_back(e);}
			virtual bool removedVertices(const graphmatch::VertexPlace& v){
				for(size_t i = 0 ; i < _removed_set_vertex.size() ; i++){
					if(v == _removed_set_vertex[i]){
						return false;
					}
				}
				return true;
			}
			virtual bool removedEdges(const graphmatch::EdgePlace& e){
				for(size_t i = 0 ; i < _removed_set_edge.size() ; i++){
					if(e == _removed_set_edge[i]){
						return false;
					}
				}
				return true;
			}
			
			/**
			 * @brief return the GraphPlace to its original state before the modifications
			 * 
			 * @param[in] gp : GraphPlace that has been modified and that one wish to return to it's previous state.
			 */
			void reset(graphmatch::GraphPlace& gp)
			{
				
	// 			std::cout << "reseting now" << std::endl; 
				std::deque<graphmatch::EdgePlace>::iterator it;
				
				for(it = _added_edges.begin(); it != _added_edges.end(); it++){
					gp.removeEdge(*it);
				}
				
				_removed_set_edge.clear(); 
				_removed_set_vertex.clear(); 
				_added_edges.clear();
				
			}
			
			/**
			 * @brief Reduce a full hypothesis of a graph as one vertices while keeping the connectivity of the graph.
			 * 
			 * Add all vertex in list h, to the removed vertex. While doing so every connection of the graph is kept.
			*It implies :
			* * Get every adjacent vertex
			* * Add a new edge between every vertex adjacent
			* * Add every new edge to the list of new adjacent _added_edges
			* * Remove vertex by adding it to _removed_set_vertex
			* 
			* @param[in] h : hypothese to reduce
			* @param[in] gp : GraphPlace we are working on 
			*/
			virtual void reduce(const std::deque< graphmatch::VertexPlace >& h, graphmatch::GraphPlace& gp);
			virtual void removeAllFalseLabel(const graphmatch::Filtered_place& fgraph, graphmatch::GraphPlace& gp);
			virtual void drawSpecialFiltered(const graphmatch::Filtered_place& fp, cv::Mat& m) const;
			virtual void drawSpecialFiltered(const graphmatch::Filtered_place& fp, cv::Mat& m, const graphmatch::VertexPlaceFiltered& v, const cv::Scalar& color) const;
			virtual void getAllEdgeLinked(const graphmatch::VertexPlace& v, const graphmatch::Filtered_place& fp, std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge) const;
			virtual void drawAll(const std::string& str, const graphmatch::Filtered_place& fp, const graphmatch::GraphPlace& gp) const;
			
			virtual void print(){
				std::cout << 
				_removed_set_edge.size() << " " <<
				_removed_set_vertex.size() << " " <<
				_added_edges.size() << std::endl;
			}
			
		};
		
	}
	
}

#endif
		