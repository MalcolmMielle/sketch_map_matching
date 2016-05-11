#ifndef VARIANCEEDGENUMBERPROBABILISTICEDITDISTANCE_MAP
#define VARIANCEEDGENUMBERPROBABILISTICEDITDISTANCE_MAP

//Put this into the graph nodes !! 

#include "MapComparator/GraphPlace.hpp"

#ifdef UNIT_TEST

namespace unit_test {   // Name this anything you like
	struct VarianceTester; // Forward declaration for befriending
}

#endif

namespace AASS{
		
	namespace probabilisticmatching{
		
		/**
		 * @brief Calculate the variance^2 associated to each vertex and add it to the graph.
		 * 
		 * The class store the fixed variance, the mean, all the custom factors and the custom variances.
		 * 
		 * the custom variance are as well added to the graph as attributes.
		 */ 
		
		class VarianceEdgeNumber{
			
		//Friend for unit testing of private function
		#ifdef UNIT_TEST
			friend struct unit_test::VarianceTester;
		#endif
			
		protected :
			
			///@brief all variance 
			std::vector<double> _v_map;
			
			///@brief factor equal to number of similar node over number of nodes. It is used to change the vartiance depending on the uniqueness.
			std::vector<double> _factor;
			
// 			std::vector<double> _Vk; <- Check out KernelDMV
	
			///@brief mean of all edge values
			double _mean;
			
			///@brief Fixed variance of the set of edges.
			double _fixed_variance;
			
			///@brief Two node are considered similar when nb of edge is in between number of edge + and - marge
			int _epsilon;

			
		public :
			
			VarianceEdgeNumber() : _mean(0), _fixed_variance(0), _epsilon(1) {};
			
			virtual void calculate(graphmatch::GraphPlace& gp);
			virtual void print();
			virtual void setEpsilon(int e){_epsilon = e;}
			virtual int getEpsilon(){return _epsilon;}
			
			std::vector<double> getVariance(){return _v_map;}
			const std::vector<double>& getVariance() const {return _v_map;}
			
			double getMean() const {return _mean;}
			double getFixedVariance() const {return _fixed_variance;}
			const std::vector<double>& getFactors() const {return _factor;}
			
			double operator[](int i){return _v_map[i];}
			const double& operator[](int i) const {return _v_map[i];}

			void clear(){
				_v_map.clear();
				_factor.clear();
				_mean = 0 ;
				_fixed_variance = 0 ;
			}
			
		protected :
			
			virtual void setNumberSimilar(graphmatch::GraphPlace& gp);
			
			virtual void setMean(AASS::graphmatch::GraphPlace& gp);
			
			///@brief get the variance with the factor
			virtual void setFixedVariance(graphmatch::GraphPlace& gp);
			
		};
		
		
		
		
		inline void VarianceEdgeNumber::calculate(AASS::graphmatch::GraphPlace& gp)
		{
			
			setFixedVariance(gp);
			setNumberSimilar(gp);
			
			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			int i = 0 ;
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				
				AASS::graphmatch::VertexPlace v = *vp.first;
				
				//Store the custom variance
				gp[v].setVariance(_fixed_variance * _factor[i]);
				_v_map.push_back(_fixed_variance * _factor[i]);
				
				i++;
				
			}

		}


		inline void VarianceEdgeNumber::setFixedVariance(AASS::graphmatch::GraphPlace& gp)
		{
			
			setMean(gp);
			
			_fixed_variance = 0 ;
			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				
				AASS::graphmatch::VertexPlace v = *vp.first;
				_fixed_variance = _fixed_variance + ( ( gp.getNumEdges(v) - _mean ) * ( gp.getNumEdges(v) - _mean ) );
				
			}
			
			_fixed_variance = _fixed_variance / (double) gp.getNumVertices(); 

		}

		inline void VarianceEdgeNumber::setMean(AASS::graphmatch::GraphPlace& gp)
		{
			
			_mean = 0 ;
			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				AASS::graphmatch::VertexPlace v = *vp.first;
				_mean = _mean + gp.getNumEdges(v);
			}
			_mean = _mean / (double) gp.getNumVertices();

		}

		inline void VarianceEdgeNumber::setNumberSimilar(AASS::graphmatch::GraphPlace& gp)
		{
			
			std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
			//vertices access all the vertix
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				AASS::graphmatch::VertexPlace v = *vp.first;
				
				double tmp = 0 ;
				
				std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp2;
				//vertices access all the vertix
				for (vp2 = boost::vertices(gp.getGraph()); vp2.first != vp2.second; ++vp2.first) {
					AASS::graphmatch::VertexPlace v2 = *vp2.first;
					
					if(gp.getNumEdges(v) <= gp.getNumEdges(v2) + _epsilon && gp.getNumEdges(v) >= gp.getNumEdges(v2) - _epsilon){
						
						tmp++;
					}
					
				}
				
				tmp = tmp / (double) gp.getNumVertices();
				
// 				_factor.push_back(tmp);
				_factor.push_back(1);
				
			}

		}

		
		
		inline void VarianceEdgeNumber::print()
		{
			std::cout << "Print variance " << std::endl;
			
			std::cout << "Fixed variance " << _fixed_variance << std::endl;
			
			std::cout << "Mean " << _mean << std::endl;
			
			std::cout << "Epsilon " << _epsilon << std::endl;
			
			std::cout << "V_map " ;
			
			for(size_t i = 0 ; i < _v_map.size() ; ++i){
				std::cout << " " << _v_map[i] << " ";
			}
			
			std::cout << std::endl << "factor " ;
			
			for(size_t i = 0 ; i < _factor.size() ; ++i){
				std::cout << " " << _factor[i] << " ";
			}
			std::cout << std::endl;

		}

		
		
		
	}
}



#endif