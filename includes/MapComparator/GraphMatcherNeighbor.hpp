#ifndef COMPARATORNEIGHBOR_MAP
#define COMPARATORNEIGHBOR_MAP

#include "GraphMatcherBase.hpp"
#include "MatchingFunctions.hpp"

namespace AASS{
		
	namespace graphmatch{
		
		/**
		 * @brief ICRA 2016 - graph matching algorithm
		 * 
		 * Use the circular order of edge and previous information given by previously matched vertices to match two planar graphs.
		 * 
		 * Attention is needed that the process is iterative and can be stop by a wrong vertex. See paper.
		 */
		class GraphMatcherNeighbor : public AASS::graphmatch::GraphMatcherBase{
			
		protected :
	// 		int _min_size_hypo_for_cluster ;
			
		public :
			GraphMatcherNeighbor() : GraphMatcherBase() {};
			
			virtual ~GraphMatcherNeighbor(){}
			/** 
			* @brief Planar edit distance matching.
			* 
			* Planar edit distance graph matching algorithm using the angle of neighbor vertices as information for the matching - icra 2016 
			*/
			
			bool planarEditDistanceAlgorithm(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
			bool planarEditDistanceAlgorithm(graphmatch::Hypothese& starting_seeds, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
			bool match(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model);
			
			
			/**
			* @brief Match the seed.
			* 
			* Initialize the matching seed for the planar graph matching algorithm using circular string matching
			* @param[in] the_pair : Match of vertices that need to be matched
			* @param[in] starting_seeds : Hypothese object containing all the seeds
			* @param[in] Q : Hypothese containing all the match with neighborhoods to match later. Get filled up by the function.
			* @param[in] seen_before : Hypothese containing all the match of vertices already created (with there neighbor matched and not matched <=> Q AND current hypothesis explored).
			* 
			* @return edit distance value.
			*/
			int initPlanar(const graphmatch::Match& the_pair, graphmatch::Hypothese& starting_seeds, graphmatch::Hypothese& Q, graphmatch::Hypothese& seen_before, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
			
	// 		int getMinSizeForClsuer(){return _min_size_hypo_for_cluster;}
	// 		void setMinSizeForCluster(int min){_min_size_hypo_for_cluster = min;}

				
		};


		inline bool GraphMatcherNeighbor::match(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
		{
// 			init(_map_one.getGraphPlace(), _map_model.getGraphPlace());
			return planarEditDistanceAlgorithm(gp, gp_model);
		}

		
	}
}

#endif