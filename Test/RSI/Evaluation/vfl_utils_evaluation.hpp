#ifndef SKETCHALGORITHM_VFLUTILEVALUATION_08112018
#define SKETCHALGORITHM_VFLUTILEVALUATION_08112018

#include "ConversionVFL.hpp"
#include "match.h"
#include "vf2_sub_state.h"
#include "argloader.h"

#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"
#include "LaplacianGraphMatching/GraphMatcherNeighborLaplacian.hpp"

#include "VFLConversion/VFLVisitor.hpp"

namespace AASS{
	namespace graphmatch{
		namespace evaluation{

			class RegionDestroyer : public AttrDestroyer{
			public:
				virtual void destroy(void* p){
					delete p;
				}
			};


			class RegionComparatorHeatAnchor : public AttrComparator{
			private:
				double _threshold;
			public:
				RegionComparatorHeatAnchor(double th) : _threshold(th){}

				virtual bool compatible(void *pa, void *pn){
					AASS::graphmatch::Region *a = (AASS::graphmatch::Region *)pa;
					AASS::graphmatch::Region *b = (AASS::graphmatch::Region *)pn;

                    assert(a->useHeatAnchors() == b->useHeatAnchors());
                    
                    double dist = a->compare(*b);
// 					double dist = std::abs(a->getHeatAnchors() - b->getHeatAnchors());
					return dist < _threshold;

				}
			};

			class RegionComparatorHeat : public AttrComparator{
			private:
				double _threshold;
			public:
				RegionComparatorHeat(double th) : _threshold(th){}

				virtual bool compatible(void *pa, void *pn){
					AASS::graphmatch::Region *a = (AASS::graphmatch::Region *)pa;
					AASS::graphmatch::Region *b = (AASS::graphmatch::Region *)pn;
                    
                    assert(a->useHeatAnchors() == b->useHeatAnchors());
                    
                    double dist = a->compare(*b);
// 					double dist = std::abs(a->getHeat() - b->getHeat());
					return dist < _threshold;

				}
			};

			inline std::ostream& operator<<(std::ostream& os, const bettergraph::MatchComparable<AASS::graphmatch::Region>& dt){
				os << "Vertex : " << dt.getFirst() << " " << dt.getSecond() << std::endl;
				return os;
			}

			int placeWhereIsVertex(const std::vector<AASS::graphmatch::GraphLaplacian::VertexLaplacian>& vp_vector, const AASS::graphmatch::GraphLaplacian::VertexLaplacian& vp){
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
			ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graphLaplacian2VFL(const AASS::graphmatch::GraphLaplacian& gp, std::map< AASS::graphmatch::Region*, AASS::graphmatch::GraphLaplacian::VertexLaplacian >& equivalent){

				std::vector<AASS::graphmatch::GraphLaplacian::VertexLaplacian > vp_vector;

				ARGEdit vfl_maker;



				/* Construct the graph here*/
				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
				//vertices access all the vertix
				for (vp = boost::vertices(gp.getGraph() ); vp.first != vp.second; ++vp.first) {
					AASS::graphmatch::GraphLaplacian::VertexLaplacian v = *vp.first;
					vp_vector.push_back(v);
					AASS::graphmatch::Region* place = new AASS::graphmatch::Region(gp[v]);

					equivalent.insert(std::pair<AASS::graphmatch::Region*, AASS::graphmatch::GraphLaplacian::VertexLaplacian>(place, v));

					vfl_maker.InsertNode( place );
				}

				for (size_t j = 0 ; j < vp_vector.size() ; ++j) {

					AASS::graphmatch::GraphLaplacian::VertexLaplacian v = vp_vector[j];
					AASS::graphmatch::GraphLaplacian::EdgeIteratorLaplacian out_i, out_end;
					AASS::graphmatch::GraphLaplacian::EdgeLaplacian e;

					for (boost::tie(out_i, out_end) = boost::out_edges(v, gp.getGraph() ); out_i != out_end; ++out_i) {
						e = *out_i;
						AASS::graphmatch::GraphLaplacian::VertexLaplacian targ = boost::target(e, gp.getGraph());

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

				ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graphvfl_tmp(&vfl_maker);
				graphvfl_tmp.SetNodeDestroyer(new RegionDestroyer());
				// Install the attribute comparator
				// This needs to be done only on graph g1.
				double my_threshold=0.3;
				graphvfl_tmp.SetNodeComparator(new RegionComparatorHeat(my_threshold));

				return graphvfl_tmp;

			}

			bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region*> > matchTest(ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType>& graph_vfl, ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType>& graph_vfl_model){

				//Graph subgraph isomorphisme using VF2
				VF2SubState s0_load(&graph_vfl, &graph_vfl_model);

				std::cout << std::endl << std::endl << "Second test " << std::endl;
				std::vector<std::pair < int, int > > vec;
				if(!match(&s0_load, AASS::visitorVector, &vec)){
					std::cout << "No matching found" << std::endl;
				}

				std::cout << "Input graph : smallest " << graph_vfl.NodeCount() << std::endl;
				std::cout << "model graph : biggest " << graph_vfl_model.NodeCount() << std::endl;

				bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region*> > vflhyp;

//				bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region*> > hyp;
				for(size_t i = 0  ; i < vec.size() ; ++i){
					bettergraph::MatchComparable<AASS::graphmatch::Region*> match(graph_vfl.GetNodeAttr(vec[i].first), graph_vfl_model.GetNodeAttr(vec[i].second) );
					vflhyp.push_back(match);
				}
//	vflhyp.extractPlace(vec, graph_vfl, graph_vfl_model);

				std::cout << "Matching size " << vflhyp.size() << " and vec size " << vec.size() << std::endl;

				return vflhyp;

			}
		}
	}
}


#endif
