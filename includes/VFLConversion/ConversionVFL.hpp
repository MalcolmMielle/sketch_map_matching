#ifndef CONVERSION_GRAPH
#define CONVERSION_GRAPH

#include "GraphPlace.hpp"
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include "MapComparator/Place.hpp"
#include "VFLib/include/argraph.h"
#include "VFLib/include/argedit.h"
#include "VFLib/include/argloader.h"
#include <iostream>
#include <stdio.h>
#include "VFLwrapper.hpp"



namespace AASS{
	
	/**
	 * @brief return position of vertex vp in vector vp_vector
	 */
	inline int placeWhereIsVertex(const std::vector<graphmatch::VertexPlace>& vp_vector, const graphmatch::VertexPlace& vp){
		for(size_t i = 0 ; i < vp_vector.size() ; ++i){
			if(vp == vp_vector[i]){
				return i;
			}
		}
		return -1;
	}

	/**
	 * @brief Convert GraphPlace to a AASS::VFLGraph VFL graph
	 */
	inline AASS::VFLGraph graphPlace2VFL(const AASS::graphmatch::GraphPlace& gp){
		
		std::vector<graphmatch::VertexPlace> vp_vector;
		
		ARGEdit vfl_maker;
		/* Construct the graph here*/
		std::pair<graphmatch::VertexIteratorPlace, graphmatch::VertexIteratorPlace> vp;
		//vertices access all the vertix
		for (vp = boost::vertices(gp.getGraph() ); vp.first != vp.second; ++vp.first) {
			graphmatch::VertexPlace v = *vp.first;
			vp_vector.push_back(v);
			graphmatch::Place* place = new graphmatch::Place(gp[v]);
			vfl_maker.InsertNode( place );
		}
		
		for (size_t j = 0 ; j < vp_vector.size() ; ++j) {
			
			graphmatch::VertexPlace v = vp_vector[j];
			graphmatch::EdgeIteratorPlace out_i, out_end;
			graphmatch::EdgePlace e;
			
			for (boost::tie(out_i, out_end) = boost::out_edges(v, gp.getGraph() ); out_i != out_end; ++out_i) {
				e = *out_i;
				graphmatch::VertexPlace targ = boost::target(e, gp.getGraph());
				
				int i = placeWhereIsVertex(vp_vector, targ);
				if(i == -1){
					std::ostringstream str_test;
					str_test <<  "Vertex do not exist line " << __LINE__ << " in file " << __FILE__;
					throw std::runtime_error( str_test.str() );
				}
				else{
					vfl_maker.InsertEdge(j, i, NULL);
				}
				
			}
		}
		
		
		
		AASS::VFLGraph graphvfl_tmp(&vfl_maker);
		
		return graphvfl_tmp;
		
	}

	/**
	 * @brief Convert ARGraph<graphmatch::Place, Empty> VFL graph to a GraphPlace
	 * Attention : it will write a temporary file !!Make sure it's not overwriting anything
	 */
	inline AASS::graphmatch::GraphPlace VFL2GraphPlace(AASS::VFLGraph& vfl){
		
		ofstream out("test_graphdb_tmp.txt");
		StreamARGLoader<graphmatch::Place, Empty>::write(out, vfl);
		out.close();
		
		
		ifstream in("test_graphdb_tmp.txt");
		AASS::graphmatch::GraphPlace gp;
		gp.read(in);
		in.close();
		
		if( remove( "test_graphdb_tmp.txt" ) != 0 ){
			perror( "Error deleting file" );
		}
		else{
			puts( "File successfully deleted" );
		}
		
		return gp;
	}

	
	
	
}

#endif