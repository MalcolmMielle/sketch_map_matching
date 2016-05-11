#ifndef COMPARATOR_HYPOTHESECLUST_MAP
#define COMPARATOR_HYPOTHESECLUST_MAP

#include "Hypothese.hpp"
#include "GraphPlace.hpp"
#include "StorageFilter.hpp"
#include "boost/bind.hpp"

namespace AASS{
	
	namespace graphmatch{
		/**
		 * @brief Cluster represente a set of possible Hypothese compatible with each others.
		 * 
		 * Deque of Hypothese that might define a graphmatch::GraphPlace matching
		 * 
		 */
		
		class Cluster{
			
		protected :
			
			/** @brief deque of Hypothese of the cluster
			*/
			std::deque< graphmatch::Hypothese > _cluster;
			
	// 		/** @brief graph input
	// 		 */
	// 		graphmatch::GraphPlace _gp;
	// 		
	// 		/** @brief graph model
	// 		 */
	// 		graphmatch::GraphPlace _gp_model;
			
			/** @brief Storage of reduced version of graph input
			*/
			StorageFilter _sf;
			
			/** @brief Storage of reduced version of graph model
			*/
			StorageFilter _sf_model;
			
				
		public :
			
			Cluster() {};
			Cluster(const std::deque< graphmatch::Hypothese >& h) : _cluster(h){};
			
			const std::deque< graphmatch::Hypothese >& getCluster(){return _cluster;}
			
			//this make a copy ?
	// 		void setGraph(const graphmatch::GraphPlace& g){ _gp = g;}
	// 		void setGraphModel(const graphmatch::GraphPlace& g){ _gp_model = g;}
			size_t size(){return _cluster.size();}
			void push_back(const graphmatch::Hypothese& pair){_cluster.push_back(pair);}
			void push_front(const graphmatch::Hypothese& pair){_cluster.push_front(pair);}
			void pop_back(){_cluster.pop_back();}
			void pop_front(){_cluster.pop_front();}
			void clear(){_cluster.clear();}
			
			const std::deque< graphmatch::Hypothese >& getCluster() const {return _cluster;}
			
			/**
			* @brief test if an Hypothese is compatible (vertex wise and geometrically consistent wise)) with the cluster
			*
			* @param[in] hyp_in : Hypothese to test
			* @param[in] gp : graphmatch::GraphPlace input
			* @param[in] gp_model : graphmatch::GraphPlace model
			* 
			* @return true if compatible, false otherwise.
			*/
			bool isCompatible(graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
			
			/**
			* @brief test if an Hypothese is has or not vertices in commun with the cluster
			*
			* @param[in] h : Hypothese to test
			* 
			* @return true if no vertices are in commun, false otherwise.
			*/
			bool isCompatible(const Hypothese h) const;
			
			/** @brief Remove all vertex outside of hypotheses
			* 
			* Remove all vertex outside of input Hypothesis and cluster while keeping the general connectivity of the graph. 
			* If two hypothesis where connected by a line of vertices, those two hypothesis are now directly connected.
			* 
			* @param[in] hyp_in : Hypothese tested for compatibility woth Cluster
			* @param[in] gp : graphmatch::GraphPlace input
			* @param[in] gp_model : graphmatch::GraphPlace model
			* @param[in] fp : graphmatch::Filtered_place input linked to _sf
			* @param[in] fp_model : graphmatch::Filtered_place model linked to _sf_model
			*/
			void reduce(graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model, const graphmatch::Filtered_place& fp, const graphmatch::Filtered_place& fp_model);
			
			/** @brief Make sure that the hypothesis and the cluster lead to both graph being equivalent
			* 
			* By looking at all the edges in between vertices of the cluster and the hypothesis tested, the function return true if the matching leads to equivalent graph for input and model and false otherwise
			* 
			* @param[in] hyp_in : Hypothese tested for compatibility woth Cluster
			* @param[in] gp : graphmatch::GraphPlace input
			* @param[in] gp_model : graphmatch::GraphPlace model
			* @param[in] fp : graphmatch::Filtered_place input linked to _sf
			* @param[in] fp_model : graphmatch::Filtered_place model linked to _sf_model
			* 
			* @return true if the matching leads to equivalent graph for input and model and false otherwise
			*/
			bool isSame(const graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::Filtered_place& fp, graphmatch::GraphPlace& gp_model, graphmatch::Filtered_place& fp_model) const;

			/**
			* @brief Make sure that any of the vertices in the Match are not already in the cluster
			* 
			* @param[in] pair : Match we wish to know if it's in the Cluster
			* @return true if the Match is in the Cluster, false otherwise.
			*/
			bool isSeen(const graphmatch::Match& pair) const;
			
			graphmatch::Hypothese& operator[](const int i){return _cluster[i];}
			const graphmatch::Hypothese& operator[](const int i) const {return _cluster[i];}
			
			/**
			* @brief DONOT USE
			* 
			* TODO REMOVE
			*/
			void reduceTEST(graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model);
			
		};
	}	
}

#endif