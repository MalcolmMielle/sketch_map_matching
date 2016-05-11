#ifndef GLOBALMATCH_MAP
#define GLOBALMATCH_MAP

#include "boost/graph/filtered_graph.hpp"

#include "Gateway.hpp"
#include "bettergraph/SimpleGraph.hpp"
#include "boost/function.hpp"

namespace AASS{
	
	namespace graphmatch{
		
		class Place;
				
		typedef typename bettergraph::SimpleGraph<Place, Gateway>::GraphType GraphType;
		typedef typename bettergraph::SimpleGraph<Place, Gateway>::Vertex VertexPlace;
		typedef typename bettergraph::SimpleGraph<Place, Gateway>::Edge EdgePlace;
		typedef typename bettergraph::SimpleGraph<Place, Gateway>::VertexIterator VertexIteratorPlace;
		typedef typename bettergraph::SimpleGraph<Place, Gateway>::EdgeIterator EdgeIteratorPlace;
		
		//Filtered graph typename
		typedef typename boost::filtered_graph<bettergraph::SimpleGraph<Place, Gateway>::GraphType, boost::keep_all, boost::function<bool(VertexPlace)> > Filtered_place;
		typedef typename boost::graph_traits< Filtered_place>::vertex_iterator VertexIteratorPlaceFiltered;
		typedef typename boost::graph_traits< Filtered_place>::vertex_descriptor VertexPlaceFiltered;
		typedef typename boost::graph_traits< Filtered_place>::edge_descriptor EdgePlaceFiltered;
		typedef typename boost::graph_traits< Filtered_place>::out_edge_iterator EdgeIteratorPlaceFiltered;

	}
	
}

#endif