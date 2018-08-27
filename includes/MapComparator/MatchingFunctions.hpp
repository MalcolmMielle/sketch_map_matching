#ifndef UTILSTOPOLO_MALCOLM_GRAPH
#define UTILSTOPOLO_MALCOLM_GRAPH

#include "GraphPlace.hpp"
#include "TopologicalMap/GraphLine.hpp"
#include "GlobalMatch.hpp"
#include <stdlib.h>     /* srand, rand */
#include <time.h>

#include "Init.hpp"
#include "bettergraph/conversion.hpp"

namespace AASS{

	namespace graphmatch{	
		
		void createMatch(const std::string& operation, 
						const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge, 
						const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge_other_graph, 
						std::pair < int, int > start,
						std::deque< graphmatch::Match >& out);
		
		void getString(size_t start, size_t end, const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge, const graphmatch::GraphPlace& gp, std::string& all_strings);
		
		/**
		 * @brief return all nodes between start and end
		 */
		void getNeighborBetween2(size_t start, size_t end, const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge, const graphmatch::GraphPlace& gp, std::deque<graphmatch::VertexPlace>& neighbor, std::deque<Place>& places);
		
		/**
		* @brief Create the string while taking in account the previous match
		*/
		double makeMatching(const graphmatch::VertexPlace& v, const graphmatch::VertexPlace& v_model, const AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::GraphPlace& gp_model, const std::deque< graphmatch::Match >& matched_previously, std::deque< graphmatch::Match >& out_match);
		
		
		void fromList2Place(const AASS::topologicalmap::GraphLine& gl, AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::AllKeypoints& allkey);
		
		///@brief add a type to every vertex using the AllKeypoints object. Need to be called to create the type. Not to be done manually.
		void addTypeVertex(AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::AllKeypoints& allkey);
		
		/*
		* 
		* 
		* 
		*/
		

		inline double makeMatching(const graphmatch::VertexPlace& v, const graphmatch::VertexPlace& v_model, const graphmatch::GraphPlace& gp, const graphmatch::GraphPlace& gp_model, const std::deque< graphmatch::Match >& matched_previously, std::deque< graphmatch::Match >& out_match){
			
	// 		std::cout << std::endl << std::endl << " MAKE MATCHING " << std::endl << std::endl;
			//List of the final string combined.
			//Get clockwise edges
			std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > > all_edge;
			gp.getAllEdgeLinkedCounterClockWise(v, all_edge);
			
	// 		std::cout << "Result for real " << std::endl;
	// 		for(size_t i = 0; i < all_edge.size() ; i++){
	// 			gp.print(all_edge[i].second);
	// 		}
			
			std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > > all_edge_model;
			gp_model.getAllEdgeLinkedCounterClockWise(v_model, all_edge_model);
			
	// 		std::cout << "Result for model " << std::endl;
	// 		for(size_t i = 0; i < all_edge_model.size() ; i++){
	// 			gp_model.print(all_edge_model[i].second);
	// 		}
			
			//ATTENTION : I feel like this could mix
			//Got to first match vertex :
			//All vertex in the neighborhood that were already matched
			std::deque< std::pair < int, int > > pair_matched;
			
			//For all node in neighbor
			for(size_t i = 0 ; i < all_edge.size() ; i++){
				
				//For all node matched before
				for(size_t w = 0 ; w < matched_previously.size() ;  w++){
					
					//If the node has been matched
					if(all_edge[i].second == matched_previously[w].getFirst()){
						
						//For all node around node in model
						for(size_t j = 0 ; j < all_edge_model.size() ; j ++){
							
							//If the node as been matched to the other side of the same matching from before
							if(all_edge_model[j].second == matched_previously[w].getSecond()){
								
								//Get in because it was alreayd matched
								pair_matched.push_back(std::pair<int, int>(i, j));
								
							}
						}
					}
				}
				
			}
			
	// 		std::cout << "found first match and there is  " << pair_matched.size() << std::endl;
			
			//Edit distance of the neighbor
			double edit_distance = 0;
			
			//If more than one match
			if(pair_matched.size() > 1){
	// 			std::cout << "IF MORE THAN ONE : " << std::endl;
				
				//Get all circular combinaison with the lock position of the neighbor thanks the already matched nodes
				for(size_t i = 0 ; i < pair_matched.size() - 1 ; i++){
// 					std::string all_strings_first;
// 					std::string all_strings_model;
// 					
// 					getString(pair_matched[i].first, pair_matched[i+1].first, all_edge, gp, all_strings_first);
					std::deque<graphmatch::VertexPlace> first_neigh;
					std::deque<Place> f_neigh_place;
					getNeighborBetween2(pair_matched[i].first, pair_matched[i+1].first, all_edge, gp, first_neigh, f_neigh_place);				
					
// 					getString(pair_matched[i].second, pair_matched[i+1].second, all_edge_model, gp_model, all_strings_model);
					std::deque<graphmatch::VertexPlace> first_neigh_model;
					std::deque<Place> f_neigh_place_model;
					getNeighborBetween2(pair_matched[i].second, pair_matched[i+1].second, all_edge_model, gp_model, first_neigh_model,f_neigh_place_model);
					
	// 				std::cout << "String result size " << all_strings_first.size() << std::endl;
					
					//Match every string correspondance
					std::string out;
				
					std::function<bool(Place, Place)> compareFunction = graphmatch::comparePlace;
					edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance<Place, Place>(f_neigh_place, f_neigh_place_model, compareFunction, out);
					
// 					edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance(all_strings_first, all_strings_model, out);
					
					//Create Match
					createMatch(out, all_edge, all_edge_model, pair_matched[i], out_match);
				}
				//Last loop
// 				std::string all_strings_first;
// 				std::string all_strings_model;
// 				getString(pair_matched[pair_matched.size()-1].first, pair_matched[0].first, all_edge, gp, all_strings_first);
// 				getString(pair_matched[pair_matched.size()-1].second, pair_matched[0].second, all_edge_model, gp_model, all_strings_model);
				
				std::deque<graphmatch::VertexPlace> first_neigh;
				std::deque<Place> f_neigh_place;
				getNeighborBetween2(pair_matched[pair_matched.size()-1].first, pair_matched[0].first, all_edge, gp, first_neigh, f_neigh_place);				
				
				std::deque<graphmatch::VertexPlace> first_neigh_model;
				std::deque<Place> f_neigh_place_model;
				getNeighborBetween2(pair_matched[pair_matched.size()-1].second, pair_matched[0].second, all_edge_model, gp_model, first_neigh_model,f_neigh_place_model);
				
// 				std::cout << "String result size " << all_strings_first.size() << std::endl;
				
				//Match every string correspondance
				std::string out;
			
				std::function<bool(Place, Place)> compareFunction = graphmatch::comparePlace;
				edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance<Place, Place>(f_neigh_place, f_neigh_place_model, compareFunction, out);
				
	// 			std::cout << "String result size " << all_strings_first.size() << std::endl;
				
				//Match every string correspondance
// 				std::string out;
// 				edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance(all_strings_first, all_strings_model, out);
				std::cout << "NOT GOOD " << std::endl;
				
				//Create Match
				createMatch(out, all_edge, all_edge_model, pair_matched[pair_matched.size()-1], out_match);
				
				
			}
			else if(pair_matched.size() == 1){
				
	// 			std::cout << "IF just one we do this : " << std::endl;
// 				std::string all_strings_first;
// 				std::string all_strings_model;
// 				getString(pair_matched[0].first, pair_matched[0].first, all_edge, gp, all_strings_first);
// 				getString(pair_matched[0].second, pair_matched[0].second, all_edge_model, gp_model, all_strings_model);
				
				std::deque<graphmatch::VertexPlace> first_neigh;
				std::deque<Place> f_neigh_place;
				getNeighborBetween2(pair_matched[0].first, pair_matched[0].first, all_edge, gp, first_neigh, f_neigh_place);				
				
				std::deque<graphmatch::VertexPlace> first_neigh_model;
				std::deque<Place> f_neigh_place_model;
				getNeighborBetween2(pair_matched[0].second, pair_matched[0].second, all_edge_model, gp_model, first_neigh_model,f_neigh_place_model);
				
// 				std::cout << "String result size " << all_strings_first.size() << std::endl;
				
				//Match every string correspondance
				std::string out;
			
				std::function<bool(Place, Place)> compareFunction = graphmatch::comparePlace;
				edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance<Place, Place>(f_neigh_place, f_neigh_place_model, compareFunction, out);
				
				
	// 			std::cout << "the linked" << std::endl;
	// 			for(size_t test = 0 ; test < all_edge.size() ; test++){
	// 				gp.print(all_edge[test].second);
	// 			}
	// 			std::cout << "the linkedmodel " << std::endl;
	// 			for(size_t test = 0 ; test < all_edge_model.size() ; test++){
	// 				gp_model.print(all_edge_model[test].second);
	// 			}
				
	// 			std::cout << "String result : " << all_strings_first << std::endl;
	// 			std::cout << "String result model :" << all_strings_model << std::endl;
				
				//Match every string correspondance
// 				std::string out;
// 				std::cout << "Normalized edit dist" << std::endl;
// 				edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance(all_strings_first, all_strings_model, out);
// 				std::cout << "End edit dist" << std::endl;
// 				std::cout << "Distance : " << edit_distance << " with " << out << " for " << all_strings_first << " and " << all_strings_model <<   std::endl;
				//Create Match
				createMatch(out, all_edge, all_edge_model, pair_matched[0], out_match);
				
	// 			std::cout << "CREATED THE MATCHES : " << out_match.size() << std::endl;
				
				
			}
			else if(pair_matched.size() == 0){
				std::cout << "NOTHING WAS ALREADY MATCHED AND THIS IS A BUG IN TOPOLOGICALMAPUTILS IN MAKEMATCHING " << std::endl;
				
			}
			
// 			std::cout << "Final distance " << edit_distance << std::endl << std::endl;
			return edit_distance;
			
		}
		
		inline void getNeighborBetween2(size_t start, size_t end, const std::deque< std::pair< AASS::graphmatch::EdgePlace, AASS::graphmatch::VertexPlace > >& all_edge, const AASS::graphmatch::GraphPlace& gp, std::deque< AASS::graphmatch::VertexPlace >& neighbor, std::deque< AASS::graphmatch::Place >& places)
		{
			neighbor.clear();
			places.clear();
			
			//Move onto first vertex to match
			start++;
			if(start == all_edge.size()){
				start = 0 ;
			}
			
			while(start != end){
				neighbor.push_back(all_edge[start].second);
				places.push_back(gp[all_edge[start].second]);
				//Move forward
				start++;
				if(start == all_edge.size()){
	// 				std::cout << "INIT TO 0 " << std::endl;
					start = 0 ;
				}
			}

		}

		
		inline void getString(size_t start, size_t end, 
						const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge,
						const graphmatch::GraphPlace& gp,
						std::string& all_strings){
			
			std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > > temp;

			size_t big = start;
			size_t small = end;
			if(big < small){
				int tmp = small;
				small = big;
				big = tmp;
			}
			
			size_t first_num =  start;
			first_num++;
			if(first_num == all_edge.size()){
	// 			std::cout << "INIT TO 0 " << std::endl;
				first_num = 0 ;
			}
	// 		std::cout << "PUSHING EDGE for " << first_num << " and " <<all_edge.size() << "and end is " << end <<std::endl;
			//TODO : Make sure this work fine
			while(first_num != end){

				//Pushing first match
	// 			std::cout << "PUSHING EDGE for " << first_num << " and " <<all_edge.size() << "and end is " << end+1 <<std::endl;
				temp.push_back(all_edge[first_num]);
				//Move forward
				first_num++;
				if(first_num == all_edge.size()){
	// 				std::cout << "INIT TO 0 " << std::endl;
					first_num = 0 ;
				}
				
			}
	// 		std::cout << "Size of before makestring : " << temp.size() << std::endl;
			all_strings = gp.makeString(temp);
			//Clear all
			temp.clear();
				
		}
		
		inline void createMatch(const std::string& operation, const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge, const std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge_other_graph, std::pair< int, int > start, std::deque< graphmatch::Match >& out){
			
			size_t place_index = start.first + 1;
			size_t model_index = start.second + 1;
			if(place_index == all_edge.size()){
				place_index = 0 ;
			}
			if(model_index == all_edge_other_graph.size()){
				model_index = 0 ;
			}
			for(size_t i = 0 ; i < operation.size(); i++){
				
				//Match
				if(operation.at(i) == 'n'){
					
					out.push_back(
						graphmatch::Match(
							all_edge[place_index].second, 
							all_edge_other_graph[model_index].second)
								);
					
					place_index++;
					model_index++;
				}
				//Substitution
				else if(operation.at(i) == 's'){
					
					out.push_back(
						graphmatch::Match(
							all_edge[place_index].second, 
							all_edge_other_graph[model_index].second)
								);
					
					place_index++;
					model_index++;
				}
				
				//TODO : find a good way to cope with deletion.
				//Deletion
				else if(operation.at(i) == 'd'){
					//needs to stay at the same place on the model but we move onto the other word
					place_index++;
				}
				//Insertion
				else if(operation.at(i) == 'i'){
					//does nothing
					model_index++;
				}
				
				
				if(place_index == all_edge.size()){
					place_index = 0 ;
				}
				if(model_index == all_edge_other_graph.size()){
					model_index = 0 ;
				}
				
				
			}
			
		}

		
		inline void fromList2Place(const AASS::topologicalmap::GraphLine& gl_in, AASS::graphmatch::GraphPlace& gp, const AllKeypoints& allkey){
			
			
// 			bettergraph::toSimpleGraph(const PseudoGraph<VertexType, EdgeType>& input, SimpleGraph<VertexType, EdgeType>& output);
			bettergraph::SimpleGraph<topologicalmap::NodeLine, vodigrex::SimpleEdge> gl;
			bettergraph::toSimpleGraph<topologicalmap::NodeLine, vodigrex::SimpleEdge>(gl_in, gl);
			
// 			std::cout << "START" << std::endl;
			try{				
				
				std::deque < std::pair <topologicalmap::GraphLine::Vertex, graphmatch::VertexPlace> > dvp;

				
				std::pair<topologicalmap::GraphLine::VertexIterator, topologicalmap::GraphLine::VertexIterator> vp;
				//vertices access all the vertices
				for (vp = boost::vertices(gl.getGraph()); vp.first != vp.second; ++vp.first) {
					topologicalmap::GraphLine::Vertex v = *vp.first;
					graphmatch::VertexPlace vpp;
//					std::vector< cv::Point > contour;
					cv::Moments mom;
					cv::Point center_mass;
					center_mass.x = gl[v].getX();
					center_mass.y = gl[v].getY();
// 					std::cout << "Ading Vertex" << std::endl;
					Place p;
//					p.contour = contour;
					p.moment = mom;
					p.mass_center = center_mass;
					
					gp.addVertex(vpp, p);
					dvp.push_back(std::pair<topologicalmap::GraphLine::Vertex, graphmatch::VertexPlace>(v, vpp) );
// 					std::cout << "done Adding" << std::endl;
				}
				
				//copy all edges and add room or not
				for (vp = boost::vertices(gl.getGraph()); vp.first != vp.second; ++vp.first) {
// 					std::cout << "Edge and rooms" << std::endl;
					topologicalmap::GraphLine::Vertex v = *vp.first;
					std::deque< std::pair< topologicalmap::GraphLine::Edge, topologicalmap::GraphLine::Vertex > > all_edges;
					gl.getAllEdgeLinked(v, all_edges);
					
					graphmatch::VertexPlace dad;
					for(size_t i = 0 ; i < dvp.size() ; i++){
						if(dvp[i].first == v){
							dad = dvp[i].second;
						}
					}
					
					//Add all edges
					for(size_t j = 0 ; j < all_edges.size() ; j++){
						graphmatch::VertexPlace son;
						for(size_t i = 0 ; i < dvp.size() ; i++){
							if(all_edges[j].second == dvp[i].first){
								son = dvp[i].second;
							}
						}
						EdgePlace edge_out;
						graphmatch::Gateway g;
						gp.addEdge(edge_out, dad, son, g);
					}
						
				}
				
				
				addTypeVertex(gp, allkey);
			}
			catch(const std::exception &e){
				std::cout << "problem during graph extraction of place in ListToPLace : " << e.what() << std::endl;
			}
		}
		
		
		inline void addTypeVertex(GraphPlace& gp, const AllKeypoints& allkey)
		{
// 				AllKeypoints allkey;
// 				std::cout << "EXTRACTING TYPE OF VERTICES........................................................." << std::endl;
			
			std::pair<VertexIteratorPlace, VertexIteratorPlace> vp;
			
			for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
				
				VertexPlace v = *vp.first;
// 					std::cout << "Size : " << allkey.getAll().size() << std::endl;
				for(size_t i = 0 ; i < allkey.getAll().size() ; i++){
						
// 						std::cout << "AT ";
// 						gp.print(v);
// 						std:: cout << allkey[i]->getID() << std::endl;
					
					auto kp = allkey[i]->compare(v, gp.getGraph());
					std::string type = gp[v].getType();
// 						std::cout << "TYPE : " << type << std::endl;
					if( kp != NULL &&  type == "notype"){
// 							std::cout << "setting the vertice" << std::endl;
						gp[v].setKeypoint(kp);
					}
// 						std::cout << std::endl;
				}
// 					std::cout << "New type : " << gp[v].getType() << std::endl;
			}
// 				std::cout << "OVER THIS " << std::endl;
			
		}

		
	}
}

#endif