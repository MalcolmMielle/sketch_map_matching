#ifndef PROBABILITIES_PFEDGE_MAP
#define PROBABILITIES_PFEDGE_MAP

#include "BaseProba.hpp"
#include "VarianceEdgeNumber.hpp"
#include <boost/math/distributions/normal.hpp>

namespace AASS{
	namespace probabilisticmatching{
		
		/**
		 * @brief table class to store and calculate edge related probabilities P[F,F].
		 */
		class PFEdgeNumber : public BaseProba{
		protected :
			
			std::vector < graphmatch::VertexPlace > _input;
			std::vector < double > _table;
			int _epsilon;
			
		public :
			PFEdgeNumber() : _epsilon(0){}
			
			virtual void setInput(const graphmatch::GraphPlace& gp);
			virtual void clear(){
				_input.clear();
			}
			virtual size_t size(){return _table.size();}
			virtual void setProbabilityTable(const AASS::graphmatch::GraphPlace& gp, const VarianceEdgeNumber& variance_input);
			
			double getProba(const graphmatch::VertexPlace& v1){
				for(size_t i = 0 ; i < _input.size() ; i++){
					if(_input[i] == v1){
						return _table[i];
					}
					
				}
				return -1;
			}
			
		protected:
			virtual void setProbabilityTable(const AASS::graphmatch::GraphPlace& gp, const VarianceEdgeNumber& variance, std::vector< AASS::graphmatch::VertexPlace >& in);
			
		};
		
		
		
		inline void PFEdgeNumber::setInput(const graphmatch::GraphPlace& gp)
		{
			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				
				AASS::graphmatch::VertexPlace v = *vp.first;
				_input.push_back(v);
				
			}
		}

		
		inline void PFEdgeNumber::setProbabilityTable(const AASS::graphmatch::GraphPlace& gp,const VarianceEdgeNumber& variance_input)
		{
			clear();
			setInput(gp);
			
			setProbabilityTable(gp, variance_input, _input);


		}
		
		//TODO Change it for a binomial
// 		inline void PFEdgeNumber::setProbabilityTable(const AASS::graphmatch::GraphPlace& gp, const AASS::probabilisticmatching::VarianceEdgeNumber& variance, std::vector< AASS::graphmatch::VertexPlace >& in)
// 		{
// 			for(size_t i = 0 ; i < in.size() ; i ++){
// 									
// 				double proba = normalDistribution(variance.getFixedVariance(), gp.getNumEdges(in[i]), variance.getMean());
// 				if(proba > 1 || proba < 0){
// 					throw std::runtime_error("Proba in PF more than 1 or less than 0");
// 				}
// 				_table.push_back(proba);
// 				
// 			}
// 
// 		}
		
		//TODO Change it for a binomial
		inline void PFEdgeNumber::setProbabilityTable(const AASS::graphmatch::GraphPlace& gp, const AASS::probabilisticmatching::VarianceEdgeNumber& variance, std::vector< AASS::graphmatch::VertexPlace >& in)
		{
			for(size_t i = 0 ; i < in.size() ; i ++){
									
// 				double proba = normalDistribution(variance.getFixedVariance(), gp.getNumEdges(in[i]), variance.getMean());
				double proba = 0;
				
				std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp2;
				//vertices access all the vertix
				for (vp2 = boost::vertices(gp.getGraph()); vp2.first != vp2.second; ++vp2.first) {
					AASS::graphmatch::VertexPlace v2 = *vp2.first;
					
					if(gp.getNumEdges(in[i]) <= gp.getNumEdges(v2) + _epsilon && gp.getNumEdges(in[i]) >= gp.getNumEdges(v2) - _epsilon){	
						proba++;
					}
				}
					
				
				
				proba = proba / (double) gp.getNumVertices();
				
// 				_factor.push_back(tmp);
				
				
				if(proba > 1 || proba < 0){
					throw std::runtime_error("Proba in PF more than 1 or less than 0");
				}
				_table.push_back(proba);
				
			}

		}


	}
}

#endif