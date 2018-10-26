#ifndef SKETCHALGORITHMS_MATCHLAPLACIAN_26112018
#define SKETCHALGORITHMS_MATCHLAPLACIAN_26112018

#include "GraphLaplacian.hpp"
#include "bettergraph/MatchComparable.hpp"

namespace AASS{
	namespace graphmatch{
		///@brief Two VertexPlace linked.
		class MatchLaplacian : public bettergraph::MatchComparable<AASS::graphmatch::GraphLaplacian::VertexLaplacian>{

		protected :
			double _cost = 0;


		public :
			MatchLaplacian(){};
			MatchLaplacian(const GraphLaplacian::VertexLaplacian& v, const GraphLaplacian::VertexLaplacian& vv) : bettergraph::MatchComparable<GraphLaplacian::VertexLaplacian>(v, vv){};

			void setCost(double cost){_cost = cost;}
			double getCost() const {return _cost;}

			virtual bool isBetterThan(const MatchLaplacian& in) const {

// 				std::cout << "cost : " << _cost << " in " << in.getCost() << std::endl ;
				if(_cost < in.getCost()){
					return true;
				}
				else{
					return false;
				}
			}

			///@brief return a positive value of difference between cost
			virtual bool isBetterThan(const MatchLaplacian& in, double& diff) const {

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
			virtual bool sameVertexThan(const MatchLaplacian& m) const{
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
//				bettergraph::MatchComparable<VertexLaplacian>::print();
				std::cout << "Cost : " << _cost << std::endl;
//				std::cout << "Proba : " << _proba << std::endl;
			}


			/**
* @brief compare the two first match to the last one and return true if the cost of the last one is less than both cost of the first ones.
*
* ATTENTION : If one want to compare only two matches, then into the same Match for match_original and match_maybe
* @param[in] match_maybe : a maybe second match to compare. input the same Match as for match_original to not use.
* @param[in] match_to_compare : the Match we want to know if the cost is less than match_original and match_maybe
*
* @return true if match_to_compare cost is less than match_original and match_maybe, false otherwise.
*/
//			bool bestMatch(const AASS::graphmatch::Match& match_maybe, const AASS::graphmatch::Match& match_to_compare, double& diff) const
//			{
//// 	std::cout << "cost match " << match_original.getCost() << " maybe " << match_maybe.getCost() << " to compare " << match_to_compare.getCost() << std::endl;
//
//				//Extract actual difference to know if it's zero and they are the same
//				bool better = match_to_compare.isBetterThan(*this, diff);
//				std::cout << "Diff first " << diff << std::endl;
//				//If there is a second match to compare. That first comparison is useless but it make the code clearer in my opinion.
//				if(match_maybe != *this){
//					double diff2 = 0;
//					if(better == true && match_to_compare.isBetterThan(match_maybe, diff2)){
//						std::cout << "Diff second " << diff2 << std::endl;
//						diff += diff2;
//						better = true;
//					}
//					else{
//						better = false;
//					}
//				}
//				return better;
//			}


		};

		inline std::ostream& operator<<(std::ostream& os, const AASS::graphmatch::MatchLaplacian& dt){
			os << "Vertex : " << dt.getFirst() << " " << dt.getSecond() << " Cost : " << dt.getCost() << std::endl;
			return os;
		}







	}

}






#endif