#ifndef PROBABILITIES_PFFUUEDGENUMBER_MAP
#define PROBABILITIES_PFFUUEDGENUMBER_MAP

#include "BaseProba.hpp"
#include "MapComparator/Match.hpp"
#include "MapComparator/GraphPlace.hpp"
#include "VarianceEdgeNumber.hpp"

#include <boost/math/distributions/normal.hpp>

namespace AASS{
	namespace probabilisticmatching{
		
		/**
		 * @brief table class to store and calculate edge related probabilities P[F,F| U = U].
		 */
		class PFFUUEdgeNumber : public BaseProba {
		
		protected:
			
			std::vector < graphmatch::VertexPlace > _input;
			std::vector < graphmatch::VertexPlace > _model;
			///@brief Normal table for searching by vertex
			std::vector < std::vector < double > > _table;
			
			
			

		public:
			PFFUUEdgeNumber(){};
			virtual  ~PFFUUEdgeNumber(){};
			
			virtual double getProba(const graphmatch::VertexPlace& v, const graphmatch::VertexPlace& v2) const;
			
			virtual void setX(const graphmatch::GraphPlace& gp);
			virtual void setY(const graphmatch::GraphPlace& gp);
			
			virtual void setProbabilityTable(const AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::GraphPlace& gp_model, const VarianceEdgeNumber& variance_model);
			
			virtual void clear(){
				_input.clear();
				_model.clear();
				_table.clear();
			}
						
			/**
			 * @brief store all vertex with the associated edge number
			 * 
			 * Model node is the second one in the match
			 */						
			virtual void print() const ;
			
			
			
		};
		
		
		inline void PFFUUEdgeNumber::setProbabilityTable(const AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::GraphPlace& gp_model, const VarianceEdgeNumber& variance_model)
		{
			clear();
			setX(gp);
			setY(gp_model);
			
			for(size_t i = 0 ; i < _input.size() ; i ++){
				_table.push_back(std::vector< double >());
				
				for(size_t j = 0 ; j < _model.size() ; j ++){
					
					double proba = normalDistribution(variance_model.getFixedVariance(), gp.getNumEdges(_input[i]), gp_model.getNumEdges(_model[j]));
				
					if(proba > 1 || proba < 0){
						throw std::runtime_error("Proba in PFFUU more than 1 or less than 0");
					}
					
					_table[i].push_back(proba);
					
				}
			}
			

		}
		
		
		inline void PFFUUEdgeNumber::setX(const graphmatch::GraphPlace& gp)
		{
			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				
				AASS::graphmatch::VertexPlace v = *vp.first;
				_input.push_back(v);
				
			}

		}

		inline void PFFUUEdgeNumber::setY(const graphmatch::GraphPlace& gp)
		{

			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				
				AASS::graphmatch::VertexPlace v = *vp.first;
				_model.push_back(v);
				
			}
		}
		
		inline double PFFUUEdgeNumber::getProba(const AASS::graphmatch::VertexPlace& v, const AASS::graphmatch::VertexPlace& v2) const
		{
			size_t i = 0 ;
			while (i < _input.size() && v != _input[i]){				
				i++;
			}
			size_t j = 0 ;
			while (j < _model.size() && v2 != _model[j]){				
				j++;
			}
			if(i != _input.size() && j != _model.size()){
// 				std::cout << "Got it at i " << i << " and j " << j << std::endl;
				return _table[i][j];
			}
			return -1;

		}
		
		inline void PFFUUEdgeNumber::print() const
		{
			std::cout << "Probability lookup table :" << std::endl;
			int ju = 0 ;
			for(size_t i = 0 ; i < _table.size() ; i++){
				
				for(size_t j = 0 ; j < _table[i].size() ; j++){
				
					std::cout << " " << _table[i][j] << " " << ju << "         ";
					ju++;
					
					if(ju % 5 == 0){
						std::cout << std::endl;	
					}
					
				}
				std::cout << std::endl;
				std::cout << std::endl;
				
			}

		}




		
		
		
	}
}
#endif