#ifndef COMPARATOR_HYPOTHESEMATCH_MAP
#define COMPARATOR_HYPOTHESEMATCH_MAP

#include "GlobalMatch.hpp"
#include "bettergraph/MatchComparable.hpp"

namespace AASS{
	namespace graphmatch{
		///@brief Two VertexPlace linked.
		class Match : public bettergraph::MatchComparable<VertexPlace>{
			
		protected :
			double _cost;
			double _proba;

			
		public :
			Match() : _cost(0), _proba(-1){};
			Match(const VertexPlace& v, const VertexPlace& vv) : bettergraph::MatchComparable<VertexPlace>(v, vv), _cost(0){};
			
			void setCost(double cost){_cost = cost;}
			double getCost() const {return _cost;}
			
			void setProba(double p){_proba = p;}
			double getProba() const {return _proba;}
			
			virtual bool isBetterThan(const Match& in) const {
				
// 				std::cout << "cost : " << _cost << " in " << in.getCost() << std::endl ;
				if(_cost < in.getCost()){
					return true;
				}
				else{
					return false;
				}
			}
			
			///@brief return a positive value of difference between cost
			virtual bool isBetterThan(const Match& in, size_t& diff) const {
				
// 				std::cout << "cost : " << _cost << " in " << in.getCost() << std::endl ;
				if(_cost < in.getCost()){
					diff = in.getCost() - _cost;
					return true;
				}
				else{
					diff = _cost - in.getCost();
					return false;
				}
			}
			
			///@brief return true if one of the vertex correspond
			virtual bool sameVertexThan(const Match& m) const{
// 				std::cout << "Comparing " <<_v1 << " with " << m.getFirst() << std::endl;
// 				std::cout << " And Comparing " <<_v2 << " with " << m.getSecond() << std::endl;
				if(_v1 == m.getFirst() || _v2 == m.getSecond()){
					return true;
				}
				return false;
			}
			
			
			
// 			///@brief return true if both vertex are the same
// 			virtual bool exactSameVertexThan(const Match& m) const{
// 				if(_v1 == m.getFirst() && _v2 == m.getSecond()){
// 					return true;
// 				}
// 				return false;
// 			}
			
			virtual void print(){
				bettergraph::MatchComparable<VertexPlace>::print();
				std::cout << "Cost : " << _cost << std::endl;
				std::cout << "Proba : " << _proba << std::endl;
			}
				
				
		};
		
		inline std::ostream& operator<<(std::ostream& os, const AASS::graphmatch::Match& dt){
			os << "Vertex : " << dt.getFirst() << " " << dt.getSecond() << " Cost : " << dt.getCost() << " Proba : " << dt.getProba() << std::endl;
			return os;
		}
		
	}

}






#endif