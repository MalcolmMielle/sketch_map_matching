#ifndef COMPARATORCLUSTERFILTERED_MAP
#define COMPARATORCLUSTERFILTERED_MAP

#include "GraphMatcherBase.hpp"
namespace AASS{
	namespace graphmatch{
		
		/**
		 * @brief BUGGY Graph Matching with a clustering scheme at the end.
		 * 
		 * Buggy as it uses only the circular order but not the previously matched vertices. DO NOT USE
		 */
		
		class GraphMatcherClusterFiltered : public GraphMatcherBase{
			
		protected :
			size_t _min_size_hypo_for_cluster ;
			std::deque<graphmatch::Hypothese> _hypothesis_to_cluster;
			
		public :
			GraphMatcherClusterFiltered() : GraphMatcherBase(), _min_size_hypo_for_cluster(10) {};
			
			virtual ~GraphMatcherClusterFiltered(){}
			/** 
			* @brief Cluster Hypotheses found in planar algorithm
			* 
			* @param[in] hypothesis_to_test : deque of Hypothese to cluster
			*/
			void cluster(std::deque<graphmatch::Hypothese>& hypothesis_to_test, AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model);
			
			/** 
			* @brief Planar edit distance matching.
			* 
			* Planar edit distance graph matching algorithm does not using the angle of neighbor vertices as information for the matching. only order of the non matched vertices 
			*/
			bool planarEditDistanceAlgorithm(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);	
			
			bool match(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
			
			int getMinSizeForClsuer(){return _min_size_hypo_for_cluster;}
			void setMinSizeForCluster(int min){_min_size_hypo_for_cluster = min;}
			
			virtual void reset(){
				_hypothesis_to_cluster.clear();
				GraphMatcherBase::reset();
			}

				
		};


		inline bool GraphMatcherClusterFiltered::match(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
		{
// 			init(_map_one.getGraphPlace(), _map_model.getGraphPlace());
			return planarEditDistanceAlgorithm(gp, gp_model);
		}

		
	}
}

#endif