#ifndef COMPARATORNEIGHBOR_MAP_LAPLACIAN
#define COMPARATORNEIGHBOR_MAP_LAPLACIAN

#include "GraphMatcherBaseLaplacian.hpp"
#include "HypotheseLaplacian.hpp"
//#include "MatchingFunctionsLaplacian.hpp"

namespace AASS{

	namespace graphmatch{

		/**
		 * @brief ICRA 2016 - graph matching algorithm
		 *
		 * Use the circular order of edge and previous information given by previously matched vertices to match two planar graphs.
		 *
		 * Attention is needed that the process is iterative and can be stop by a wrong vertex. See paper.
		 */
		class GraphMatcherNeighborLaplacian : public AASS::graphmatch::GraphMatcherBaseLaplacian{

		protected :
			// 		int _min_size_hypo_for_cluster ;

		public :
			GraphMatcherNeighborLaplacian() : GraphMatcherBaseLaplacian() {};

			virtual ~GraphMatcherNeighborLaplacian(){}
			/**
			* @brief Planar edit distance matching. Return true if matched + number of seeds
			*
			* Planar edit distance graph matching algorithm using the angle of neighbor vertices as information for the matching - icra 2016
			*/

			std::tuple<bool, int> planarEditDistanceAlgorithm(graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model);
			bool planarEditDistanceAlgorithm(graphmatch::HypotheseLaplacian& starting_seeds, graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model);
			bool match(AASS::graphmatch::GraphLaplacian& gp, AASS::graphmatch::GraphLaplacian& gp_model);


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
			int initPlanar(const graphmatch::MatchLaplacian& the_pair, graphmatch::HypotheseLaplacian& starting_seeds, graphmatch::HypotheseLaplacian& Q, graphmatch::HypotheseLaplacian& seen_before, graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model);

			// 		int getMinSizeForClsuer(){return _min_size_hypo_for_cluster;}
			// 		void setMinSizeForCluster(int min){_min_size_hypo_for_cluster = min;}


		};


		inline bool GraphMatcherNeighborLaplacian::match(graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model)
		{
// 			init(_map_one.getGraphPlace(), _map_model.getGraphPlace());
			auto [worked, seeds] = planarEditDistanceAlgorithm(gp, gp_model);
            return worked;
		}


	}
}

#endif
