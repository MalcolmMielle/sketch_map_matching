// #include "GraphMatcherOld.hpp"
// 
// int AASS::graphmatch::GraphMatcherOld::structuralDifferencies(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model)
// {
// 	/* Strategy :
// 		* 
// 		* Compute number of crossing and number of place of each map
// 		* Brokeness factor is number of difference in between them
// 		* 
// 		*/
// 	
// 	
// 	//PLACES
// 	int corner_count = 0;
// 	int room_count = 0 ;
// 	int corner_count_model = 0;
// 	int room_count_model = 0 ;
// 	
// 	//TODO : add corridors
// 	
// 	countingPlace(corner_count, room_count, gp);
// 	countingPlace(corner_count_model, room_count_model, gp_model);
// 	
// 	int diff_corner = corner_count - corner_count_model;
// 	int diff_room = room_count - room_count_model;
// 	
// 	if(diff_corner < 0){
// 		diff_corner = - diff_corner;
// 	}
// 	if(diff_room < 0){
// 		diff_room = - diff_room;
// 	}
// 	std::cout << "Places differencies" << std::endl << "****************************" << std::endl;;
// 	std::cout << "Model -> corner : " << corner_count_model << " rooms : " << room_count_model << std::endl;
// 	std::cout << "Map -> corner : " << corner_count << " rooms : " << room_count << std::endl;
// 	std::cout << "****************************" << std::endl;;
// 	
// 	//VORONOI
// 	int corner_count_voronoi = 0;
// 	int junction_count = 0 ;
// 	int corner_count_voronoi_model = 0;
// 	int junction_count_model = 0 ;
// 	
// 	//TODO : add corridors
// 	
// // 	countingVoronoiLines(corner_count_voronoi, junction_count, _graph_list);
// // 	countingVoronoiLines(corner_count_voronoi_model, junction_count_model, _graph_list_model);
// 	
// 	int diff_corner_voronoi = corner_count - corner_count_model;
// 	int diff_junction_voronoi = room_count - room_count_model;
// 	
// 	if(diff_corner_voronoi < 0){
// 		diff_corner_voronoi = - diff_corner_voronoi;
// 	}
// 	if(diff_junction_voronoi < 0){
// 		diff_junction_voronoi = - diff_junction_voronoi;
// 	}
// 	
// 	std::cout << "Voronoi lines differencies" << std::endl << "****************************" << std::endl;;
// 	std::cout << "Model -> corner : " << corner_count_voronoi_model << " junction : " << junction_count_model << std::endl;
// 	std::cout << "Map -> corner : " << corner_count_voronoi << " junction : " << junction_count << std::endl;
// 	std::cout << "****************************" << std::endl;;
// 	
// 	return diff_room + diff_corner + diff_corner_voronoi + diff_junction_voronoi;
// 	
// }
// 
// 
// void AASS::graphmatch::GraphMatcherOld::countingPlace(int& corner, int& rooms, graphmatch::GraphPlace& gp)
// {
// 
// 	corner = 0 ;
// 	rooms = 0;
// 	
// 	std::pair<VertexIteratorPlace, VertexIteratorPlace> vp;
// 	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; vp.first++) {
// 	
// 		VertexPlace v = *vp.first;
// 		graphmatch::Place place = gp[v];
// 		
// 		if(gp.isRoom(v) == false){
// 			corner++;
// 		}
// 		else{
// 			rooms++;
// 		}
// 				
// 	}
// 	
// 	std::cout << "Number of rooms " << rooms << std::endl;
// 	std::cout << "Number of corner " << corner << std::endl;
// 	
// 	
// }
// 
// void AASS::graphmatch::GraphMatcherOld::countingVoronoiLines(int& corners, int& junctions, const topologicalmap::GraphList& gl)
// {
// 
// 	corners = 0 ;
// 	junctions = 0;
// 	
// 	std::pair<topologicalmap::VertexIterator, topologicalmap::VertexIterator> vp;
// 	for (vp = boost::vertices(gl.getGraph()); vp.first != vp.second; vp.first++) {
// 	
// 		topologicalmap::Vertex v = *vp.first;
// 
// 		if(gl.getNumEdges(v) < 1){
// 			corners++;
// 		}
// 		else{
// 			junctions++;
// 		}
// 				
// 	}
// 	
// 	std::cout << "Number of corners " << corners << std::endl;
// 	std::cout << "Number of junction " << junctions << std::endl;
// }
// 
// 
// 
// /* See : 
// 	* 
// 	* Wesley H. Huang
// 	* Kristopher R. Beevers
// 	* 
// 	* Topological Map Matching
// 	* 
// 	*/
// 
// void AASS::graphmatch::GraphMatcherOld::growHypotheses(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp2, std::deque<std::deque< graphmatch::Match > >& H)
// {
// 	
// 	//First : single vertex matching
// 	std::deque< graphmatch::Match > places_pair;
// 	
// 	//Get all pair wise association
// 	pairWiseMatch(gp, gp2, places_pair);
// 	
// 	std::deque< graphmatch::Match > pair_of_hypo;
// 	std::deque< graphmatch::Match > pair_to_expand;
// 	
// 	std::cout << "Getting in the while" << std::endl;
// 	//Second : expension of the matching
// 	while(places_pair.empty() == false){
// 		
// 		pair_to_expand.push_back(places_pair.at(0));
// 		places_pair.pop_front();
// 		
// 		while(pair_to_expand.empty() == false){
// 			std::cout << "Pair to expend while" << std::endl;
// 			
// 			std::cout << "Working on" << std::endl;
// 			gp.printHighLevel(pair_to_expand.at(0).getFirst());
// 			gp.printHighLevel(pair_to_expand.at(0).getSecond());
// // 				int a;
// // 				std::cin >> a;
// 			
// 			pair_of_hypo.push_back(pair_to_expand.at(0));
// 			pair_to_expand.pop_front();
// 			
// 			//For all matching edge of the new hypo element
// 			std::deque < std::pair < EdgePlace, VertexPlace > > all_edge_linked_gp;
// 			std::deque < std::pair < EdgePlace, VertexPlace > > all_edge_linked_gp2;
// 			
// 			gp.getAllEdgeLinked(pair_of_hypo.at(pair_of_hypo.size() -1).getFirst(), all_edge_linked_gp);
// 			gp.getAllEdgeLinked(pair_of_hypo.at(pair_of_hypo.size() -1).getSecond(), all_edge_linked_gp2);
// 			
// 			
// 			
// 			for(size_t i = 0 ; i < all_edge_linked_gp.size() ; i++){
// 				std::cout << "Trying all edge linked" << std::endl;
// 				
// 				VertexPlace targ_gp;
// 				gp.getTarget(all_edge_linked_gp.at(i).first, targ_gp );
// 				
// 				//Avoid case where hypo is empty
// 				if(pair_of_hypo.size() > 0){
// 				
// 					for(size_t j = 0 ; j < all_edge_linked_gp2.size() ; j++){
// 						std::cout << "Comparing them to other edges" << std::endl;
// 						
// 						if(pair_of_hypo.size() > 0){
// 						
// 							VertexPlace targ_gp2;
// 							gp2.getTarget(all_edge_linked_gp2.at(i).first, targ_gp2 );
// 							
// 							std::cout << "Testing " << std::endl;
// 							gp.printHighLevel(targ_gp);		
// 							std::cout << " and " << std::endl;
// 							gp.printHighLevel(targ_gp2);
// 	// 						int a;
// 	// 						std::cin >> a;
// 							
// 							int flag = false;
// 							std::deque< graphmatch::Match >::iterator it;
// 							
// 							//if both target already belong to hyp or expand do nothing. We already have them.
// 							for(it = pair_of_hypo.begin() ; it != pair_of_hypo.end() ;){
// 	// 							std::cout << "Condition one" << std::endl;
// 								if(targ_gp == (*it).getFirst() && targ_gp2 == (*it).getSecond() ){
// 									std::cout << "True" << std::endl;
// 									gp.print((*it).getFirst());
// 									gp.print((*it).getSecond());
// 									
// 									flag = true;
// 									it++;
// 								}
// 								else{
// 									it++;
// 								}
// 							}
// 							
// 							for(it = pair_to_expand.begin() ; it != pair_to_expand.end() ;){
// 	// 							std::cout << "Condition two" << std::endl;
// 								if(targ_gp == (*it).getFirst() && targ_gp2 == (*it).getSecond() ){
// 									std::cout << "True" << std::endl;
// 									gp.printHighLevel((*it).getFirst());
// 									gp.printHighLevel((*it).getSecond());
// 									
// 									flag = true;
// 									it++;
// 								}
// 								else{
// 									it++;
// 								}
// 							}
// 							
// 							//if both target are inside the possible pair and the edge are matching
// 								//Add then to pair to expand and remove them from pair wise matching
// 							for(it = places_pair.begin() ; it != places_pair.end() ;){
// 	// 							std::cout << "Condition three" << std::endl;
// 								if(targ_gp == (*it).getFirst() && targ_gp2 == (*it).getSecond() ){
// 									
// 									std::cout << "new pair to expand" << std::endl;
// 									gp.printHighLevel((*it).getFirst());
// 									gp.printHighLevel((*it).getSecond());
// 	// 								int a;
// 	// 								std::cin >> a;
// 									
// 									pair_to_expand.push_back(*it);
// 									places_pair.erase(it);
// 									flag = true;
// 								}
// 								else{
// 									it++;
// 								}
// 							}
// 							
// 							//else
// 								//discard the hypothesis						
// 							if(flag == false){
// 								std::cout << "discard hypo with " << pair_of_hypo.size() << " point match" << std::endl;
// 																
// // 									cv::imshow("Temporal", tmp);
// // 									cv::waitKey(0);
// 								
// 								//ATTENTION : this was written in the paper but I don't understand this. It suppress every hypothesis that is not perfect basically. Instead we should keep all of them and keep the best one for later use.
// 								
// // 									pair_of_hypo.clear();
// // 									pair_to_expand.clear();	
// 								
// 								//So instead of this, let's just get out and add a new hypothesis :)
// 								
// 								H.push_back(pair_of_hypo);
// 								
// 								pair_of_hypo.clear();
// 								pair_to_expand.clear();
// 							
// 							}
// 						}
// 					}
// 				}
// 				
// 				std::cout << "... out  of edge linked list comparison of the target ..." << std::endl;
// 			}	
// 			
// 			std::cout << "... out  of edge linked list comparison ..." << std::endl;
// 		}
// 		
// 		//if Q is not empty
// 			//add Q to the valid hypothesis
// // 			if(pair_of_hypo.empty() == false){
// // 				std::cout << "Pushing new hypo" << std::endl;
// // // 				int a ;
// // // 				std::cin >> a;
// // 				
// // 				H.push_back(pair_of_hypo);
// // 			}
// // 			
// 	}
// 	std::cout << "OVER" << std::endl;
// // 		return H;
// 
// }