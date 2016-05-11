#ifndef PROBABILISTICEDITDISTANCE_MAP
#define PROBABILISTICEDITDISTANCE_MAP

#include "GraphProbaEdge.hpp"
#include "GraphPlace.hpp"
#include "Hypothese.hpp"
#include "GraphMatcherAnchor.hpp"
#include "ProbabilisticMethod/Util.hpp"
#include "MapComparator/Util.hpp"

#ifdef TIMED
#include "Timed.hpp"
#endif

namespace AASS{
		
	namespace probabilisticmatching{
		
		/**
		 * @brief Class charged to do a probabilistic graph matching using the editdistance as a measurement.
		 * To be published.
		 * 
		 */ 
		class ProbabilisticEditDistanceEdge : public graphmatch::GraphMatcherBase{
			
		protected :
			double ante_prob;
// 			std::deque<graphmatch::Hypothese> _hypothesis_final;
			graphmatch::GraphMatcherAnchor _anchor_matcher;
			
			GraphProbaEdge _graphs;
			
			
		public :
			ProbabilisticEditDistanceEdge() : ante_prob(0) {};
			
			bool match(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model);
			void init(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model);
			bool algo4(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model);
			void pushAnchor(const graphmatch::Match& m){
				_anchor_matcher.pushAnchor(m);
			}
			
// 		protected: 
			///@brief Use the probabilities to get the matching doors
			void getAnchorsInit(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp2, std::deque< AASS::graphmatch::Match >& anchor);
			/**
			 * @brief Get one more anchor using probabilities
			 * 
			 */
			void getAnchors(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp2, std::deque< AASS::graphmatch::Match >& anchor);
			
			/**
			 * @brief get all anchor with the best probabilities. _graphs need to be sorted.
			 * 
			 * @param[in] anchor : deque of all the alrerady selected getAllBestAnchors
			 * @param[in] temp_anchor : empty deque that will store all the new possible anchors.
			 */
			void getAllBestAnchors(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp2, const std::deque< AASS::graphmatch::Match >& anchor, std::deque< AASS::graphmatch::Match >& temp_anchor);
			
			void getDoors(std::deque < graphmatch::VertexPlace>& doors, graphmatch::GraphPlace& gp);
			
			
			
			
			
			
		};
		
	}
	
}

#endif