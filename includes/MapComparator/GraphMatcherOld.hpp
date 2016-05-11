// #ifndef COMPARATOROLD_MAP
// #define COMPARATOROLD_MAP
// 
// #include "GraphMatcherBase.hpp"
// namespace AASS{
// 	
// 	namespace graphmatch{
// 		
// 		/**
// 		* @brief Basic graph similarity assessment.
// 		* 
// 		* Very simple version of a graphmatch. Just compare the number of vertices and their type
// 		* 
// 		*/
// 		class GraphMatcherOld : public GraphMatcherBase{
// 			
// 
// 		public :
// 			GraphMatcherOld() : GraphMatcherBase() {};
// 						
// 			/** @brief Calculate the brokenness of model map by comparing the difference between the attributes of the graphs
// 			*/
// 			int structuralDifferencies(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
// 			
// 			/**
// 			* @param[in] corner : return number of corner in GraphPlace gp
// 			* @param[in] rooms : return number of rooms in GraphPlace gp
// 			* @param[in] gp : topologicalmap::graphPlace we work on
// 			*/
// 			void countingPlace(int& corner, int& rooms, graphmatch::GraphPlace& gp);
// 			
// 			/**
// 			* @param[in] corner : return number of corner in topologicalmap::GraphList gl
// 			* @param[in] junctions : return number of junctions in topologicalmap::GraphList gl
// 			* @param[in] gp : topologicalmap::graphPlace we work on
// 			*/
// 			void countingVoronoiLines(int& corners, int& junctions, const topologicalmap::GraphList& gl);		
// 			
// 			/**
// 			* @brief Algorithm 1. (A, B) “Grow” potential matches between two maps A and B.
// 			*/
// 			void growHypotheses(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp2, std::deque< std::deque< graphmatch::Match > >& H);
// 			
// 			virtual bool match(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
// 				
// 		};
// 		
// 		inline bool GraphMatcherOld::match(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
// 		{
// // 			init(_map_one.getGraphPlace(), _map_model.getGraphPlace());
// 			std::cout << "Strurctural differencies : " << structuralDifferencies(gp, gp_model) << std::endl;
// 			return true;
// 
// 		}	
// 	}	
// }
// 
// #endif