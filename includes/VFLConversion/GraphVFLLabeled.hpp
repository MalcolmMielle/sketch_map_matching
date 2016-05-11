#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "GraphPlace.hpp"
#include "ConversionVFL.hpp"

namespace AASS{
	
	class NodeSimple{
	public:
		int x;
		int y;
		NodeSimple() : x(0), y(0){};
	};
	
	inline std::ostream& operator<<(std::ostream& out, const NodeSimple& p){
		out << p.x;		
		return out;
	}
	
	inline std::istream& operator>>(std::istream& in, NodeSimple& p){
		in >> p.x;
		return in;
	}
	
	
	typedef boost::adjacency_list<
			boost::listS, boost::listS, boost::undirectedS, 
			NodeSimple,
			Empty, 
			boost::no_property > Graph_Labeled_tmp;
			
// 	typedef typename boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;
	typedef typename boost::graph_traits<Graph_Labeled_tmp>::vertex_iterator VertexIterator_Labeled_tmp;
	typedef typename boost::graph_traits<Graph_Labeled_tmp>::vertex_descriptor Vertex_Labeled_tmp;
	typedef typename boost::graph_traits<Graph_Labeled_tmp>::edge_descriptor Edge_Labeled_tmp;
	typedef typename boost::graph_traits<Graph_Labeled_tmp>::out_edge_iterator EdgeIterator_Labeled_tmp;
	
}
/*
namespace bettergraph{
	template<>
	class AttributeAdder<AASS::NodeSimple, AASS::Empty>{
	protected : 
		typedef boost::adjacency_list<
			boost::listS, boost::listS, boost::undirectedS, 
			AASS::NodeSimple,
			AASS::Empty, 
			boost::no_property > GraphType;
		typedef typename boost::graph_traits<GraphType>::vertex_iterator VertexIterator;
		typedef typename boost::graph_traits<GraphType>::vertex_descriptor Vertex;
		typedef typename boost::graph_traits<GraphType>::edge_descriptor EdgeType;
		typedef typename boost::graph_traits<GraphType>::out_edge_iterator EdgeIterator;
		
	public :
		AttributeAdder(){};
		virtual void add(GraphType& arg1, Vertex& vertex, const AASS::NodeSimple& vertexAttribute){
			arg1[vertex].x = vertexAttribute.x;
			arg1[vertex].y = vertexAttribute.y;
		};
		virtual void add(GraphType& arg1, EdgeType& edge, const AASS::Empty& edgeAttribute){
		};
		
	};
}*/




namespace AASS{

	/**
	*@brief Reading the file a labeled graph with only one integer and converting them to a Graph Place with zero on the second number
	* *************************/

	//Need a NodeSimple because of the read function
	inline bettergraph::SimpleGraph<NodeSimple, Empty> getLabeledGraph(std::ifstream& in){
		bettergraph::SimpleGraph<NodeSimple, Empty> gp;
		gp.read(in);
		return gp;
	}
	
	
	inline graphmatch::GraphPlace getLabeledGraph2Place(std::ifstream& in){
		
		bettergraph::SimpleGraph<NodeSimple, Empty> gp = getLabeledGraph(in);
		
// 		std::cout << "Number of vertex" << gp.getNumVertices() << std::endl;
		graphmatch::GraphPlace gp_return;
		std::vector<graphmatch::VertexPlace> vecp;
		std::vector<Vertex_Labeled_tmp> vecp_tmp;
		
		//first is beginning, second is "past the end"
		std::pair<VertexIterator_Labeled_tmp, VertexIterator_Labeled_tmp> vp;
		//vertices access all the vertix
		for (vp = boost::vertices( gp.getGraph() ); vp.first != vp.second; ++vp.first) {
// 			std::cout << "Adding vertex" << std::endl;
			Vertex_Labeled_tmp v = *vp.first;
			graphmatch::VertexPlace vp;
			graphmatch::Place place;
			place.mass_center.x = gp[v].x;
			place.mass_center.y = gp[v].y;
			gp_return.addVertex(vp, place);
			vecp.push_back(vp);
			vecp_tmp.push_back(v);

		}
		
		for (vp = boost::vertices( gp.getGraph() ); vp.first != vp.second; ++vp.first) {
			Vertex_Labeled_tmp v = *vp.first;
			EdgeIterator_Labeled_tmp out_i, out_end;
			Edge_Labeled_tmp e;
			for (boost::tie(out_i, out_end) = boost::out_edges(v, gp.getGraph()); 
				out_i != out_end; ++out_i) {
// 				std::cout << "Adding edges" << std::endl;
				e = *out_i;
				Vertex_Labeled_tmp src = boost::source(e, gp.getGraph()), targ = boost::target(e, gp.getGraph());
				
				int spec = 0 ;
				int gugu = 0 ;
				for(size_t i = 0 ; i < vecp_tmp.size() ; ++i){
					if(src == vecp_tmp[i]){
						spec = i;
					}
					if(targ == vecp_tmp[i]){
						gugu = i;
					}
				}
			
				graphmatch::EdgePlace edgeplace;
				gp_return.addEdge(edgeplace, vecp[spec], vecp[gugu]);
			
			}
			
		}
		return gp_return;
		
	}
	
	
}



