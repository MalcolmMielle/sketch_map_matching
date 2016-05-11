#ifndef PROBABILITIES_GRAPHPROBA_MAP
#define PROBABILITIES_GRAPHPROBA_MAP

#include "GraphPlace.hpp"
#include "Match.hpp"

namespace AASS{
	namespace probabilisticmatching{
		
		/**
		 * @brief table class to store and calculate edge related probabilities P[U = U | F,F].
		 */
		class GraphProba {
			
		protected :
			graphmatch::GraphPlace _gp;
			graphmatch::GraphPlace _gp_model;
			
			
		public :
			GraphProba(){}
			virtual void setInput(const graphmatch::GraphPlace g){_gp = g;}
			virtual void setModel(const graphmatch::GraphPlace g){_gp_model = g;}
			virtual graphmatch::GraphPlace& getInput(){return _gp;}
			virtual graphmatch::GraphPlace& getModel(){return _gp_model;}
			virtual const graphmatch::GraphPlace& getInput() const {return _gp;}
			virtual const graphmatch::GraphPlace& getModel() const {return _gp_model;}
			
			///@brief set all proba
			virtual void setProba(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model) = 0;
			///@brief return the probability P[U = U | F, F] of the match at position i
// 			virtual double getPUUFF(int i) = 0;
			///@brief return the probability P[U = U | F, F] of the match between v and v2
			virtual double getPUUFF(const graphmatch::VertexPlace v, const graphmatch::VertexPlace v2) = 0;
			/**
			 * @brief return the match at position i
			 * 
			 * @param[in] i : position of match
			 * @return MatchProb
			 * 
			 * */
			virtual graphmatch::Match& getMatch(int i) = 0 ;
			virtual const graphmatch::Match& getMatch(int i) const = 0 ;
			
		};
	}
}

#endif