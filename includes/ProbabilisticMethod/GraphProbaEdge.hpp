#ifndef PROBABILITIES_GRAPHPROBAEDGE_MAP
#define PROBABILITIES_GRAPHPROBAEDGE_MAP

#include "GraphProba.hpp"
#include "PFFUUEdgeNumber.hpp"
#include "PFEdgeNumber.hpp"
#include "VarianceEdgeNumber.hpp"

namespace AASS{
	namespace probabilisticmatching{
		
		/**
		 * @brief table class to store and calculate edge related probabilities P[U = U | F,F].
		 */
		class GraphProbaEdge : public GraphProba {

		protected :
			PFFUUEdgeNumber _pffuu;
			PFEdgeNumber _pf_input;
			PFEdgeNumber _pf_model;
			VarianceEdgeNumber _variance_input;
			VarianceEdgeNumber _variance_model;
			double _puu;
			
			///@brief Table of all proba given by a certain match
			std::vector < graphmatch::Match > _table_match;
			
			
		public :
			GraphProbaEdge(){};
			
			virtual void clear(){
			
				_pffuu.clear();
				_pf_input.clear();
				_pf_model.clear();
				_variance_input.clear();
				_variance_model.clear();
				_table_match.clear();
			}
			
			virtual void print(){
				for(size_t i  = 0 ; i< _table_match.size() ; i++){
					std::cout << "prob " << _table_match[i].getProba() << std::endl;
				}
			}
			
			///@brief set all proba
			virtual void setProba(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model);
			
			///@brief return the probability P[U = U | F, F] of the match between v and v2
			virtual double getPUUFF(const graphmatch::VertexPlace v, const graphmatch::VertexPlace v2);
			
			/**
			 * @brief return the match at position i
			 * 
			 * @param[in] i : position of match
			 * @return graphmatch::Match
			 * 
			 * */
			virtual const graphmatch::Match& getMatch(int i) const{return _table_match[i];}
			virtual graphmatch::Match& getMatch(int i) {return _table_match[i];}
			
			///@brief return the number of probabilities
			virtual size_t size() const {return _table_match.size();}
			
			///@brief Sort the table from the biggest to the smallest proba
			virtual void sort(std::vector< graphmatch::Match >& table_match);
			
			///@brief Sort the table from the biggest to the smallest proba
			virtual void sort(){
				sort(_table_match);
			};
			
			virtual const PFFUUEdgeNumber getPFFUUTable() const {return _pffuu;}
						
		};
		
		
		
		inline void GraphProbaEdge::sort(std::vector < graphmatch::Match >& table_match)
		{
			std::vector< graphmatch::Match >::iterator hypothesis_final_ite;
			std::vector< graphmatch::Match >::iterator hypothesis_final_ite_2;
			
			graphmatch::VertexPlace v;
			graphmatch::Match copy(v, v);
			
			for(hypothesis_final_ite =  table_match.begin()+1 ; hypothesis_final_ite !=  table_match.end() ; hypothesis_final_ite++){
				
				hypothesis_final_ite_2 = hypothesis_final_ite ;
				copy = *hypothesis_final_ite;
		// 			std::cout << "tsart" << std::endl;
		// 			std::cout << "something" <<  ( * (hypothesis_final_ite_2 - 1) ).getDist() << " < " << (*hypothesis_final_ite_2).getDist()  << std::endl;
				while( hypothesis_final_ite_2 !=  table_match.begin() && (*(hypothesis_final_ite_2 - 1)).getProba() < copy.getProba()){
					
		// 				std::cout << "MOVE" << std::endl;
					*( hypothesis_final_ite_2 ) = *( hypothesis_final_ite_2-1 );
					hypothesis_final_ite_2 = hypothesis_final_ite_2 - 1;
				
					
				}
				*(hypothesis_final_ite_2) = copy;
			}

		}
		
		inline void GraphProbaEdge::setProba(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
		{
				
			clear();
			
			_variance_input.calculate(gp);
			_variance_model.calculate(gp);
			
// 			_variance_input.print();
// 			_variance_model.print();
			
			_pffuu.setProbabilityTable(gp, gp_model, _variance_model);
			_pf_input.setProbabilityTable(gp, _variance_input);
			_pf_model.setProbabilityTable(gp_model, _variance_model);
			
			//smallest number of vertices
			int min = gp.getNumVertices();
			if(min > gp_model.getNumVertices()){
				min = gp.getNumVertices();
			}
			_puu = 1/ (double) min;
			
			std::pair<graphmatch::VertexIteratorPlace, graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				graphmatch::VertexPlace v = *vp.first;
				
				std::pair<graphmatch::VertexIteratorPlace, graphmatch::VertexIteratorPlace> vp_model;
				//vertices access all the vertix
// 				std::cout << "Creating table proba now" << std::endl;
// 				int i = 0 ;
				for (vp_model = boost::vertices(gp_model.getGraph()); vp_model.first != vp_model.second; ++vp_model.first) {
					graphmatch::VertexPlace v_model = *vp_model.first;
					
// 					std::cout << " place " << i << " " ;
// 					++i;
					
					double proba =  getPUUFF(v, v_model);
					graphmatch::Match m(v, v_model);
					m.setProba(proba);
					_table_match.push_back(m);
										
				}
				
			}
			
		}
		
		inline double GraphProbaEdge::getPUUFF(const AASS::graphmatch::VertexPlace v, const AASS::graphmatch::VertexPlace v2)
		{
				
// 				std::cout << "puu " << _puu << " " ;
// 			std::cout << "Getting pffuu " << std::endl;
			double prob_ffuu = _pffuu.getProba(v, v2);
// 				std::cout << "pffuu "<< prob_ffuu << " ";
			
// 			double proba_f = _pf_input.getProba(v);
// 				std::cout << "pf "<< proba_f << " ";
// 			double proba_f_model = _pf_model.getProba(v2);
// 				std::cout << "p_model "<< proba_f_model << " ";
			
// 			if(prob_ffuu < 0 || proba_f < 0 || proba_f_model < 0 || prob_ffuu > 1 || proba_f > 1 || proba_f_model > 1){
				
// 				throw std::runtime_error("Probabilities should be between 0 and 1");
// 			}
			
// 			proba_f = proba_f * proba_f_model;
			
// 			double proba_ff = (prob_ffuu * _puu ) + (proba_f * ( 1 - _puu ) );
// 				std::cout << "pff "<< proba_ff << " ";

			//ATTENTION NORMAL FORMULA : why do Pff lower make the probability be better ?
// 				double res = (_puu * prob_ffuu) / proba_ff;
			//ATTENTION FORMULA THAT WORK WITH THE GRAPH
			double res = prob_ffuu;

// 				std::cout << "res "<< res << " " << std::endl;
			return res;
			
		}

	}
}

#endif