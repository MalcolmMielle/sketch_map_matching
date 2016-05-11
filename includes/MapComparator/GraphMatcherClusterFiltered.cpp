#include "GraphMatcherClusterFiltered.hpp"

// 	
// 	/* Planar edit Distance graph matching
// 	 * This is from An erro tolerant approximate matching algorithm by Neuhaus.
// 	 * 
// 	 */
// 	
// 	//TODO : the comparison every time is absolutely not optimised. A better way would be to have a table with label and work with it.
// return false if no solution found
bool AASS::graphmatch::GraphMatcherClusterFiltered::planarEditDistanceAlgorithm(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
{		
	
	//Determine seed substitutions (ie which node is comaprable to which node in the other graph)
	//Add seed substitution to the FIFO queue Q
	
	graphmatch::Hypothese starting_seeds;
	graphmatch::Hypothese seen_before;
	graphmatch::Hypothese Q;
	graphmatch::Hypothese hypothesis;
			
	pairWiseMatch(gp, gp_model, starting_seeds.getMatches());
	
// 	std::cout << "Size of pair wise " << starting_seeds.size() << std::endl;
	
	//ATTENTION : Suppressed this for to cluster because it become really slow.
	for(size_t i = 0 ; i < starting_seeds.size() ; i++){
		graphmatch::Hypothese hyp;
		hyp.push_back(starting_seeds[i]);
		int newdist = hyp.updateDistance(gp, gp_model);
		hyp.setDist(newdist);
		_hypothesis_final.push_back(hyp);
	}
	//To keep all hypo by themselves
				
	while(starting_seeds.empty() == false){
				
		seen_before.clear();
		Q.push_back(starting_seeds.at(0));
		seen_before.push_back(starting_seeds.at(0));
		starting_seeds.pop_front();
		
		int dist = 0;
			
		while(Q.empty() == false){
			
// 				std::cout << " size of Q : " << Q.size() << std::endl;
		
			//Fetch next substitution from Q
			graphmatch::Match the_pair(Q[0].getFirst(), Q[0].getSecond());
			
			Q.pop_front();
			hypothesis.push_back(the_pair);
		
			//Match neighborhoods
			std::deque< std::pair< EdgePlace, VertexPlace > > all_edge;
			gp_model.getAllEdgeLinkedCounterClockWise(the_pair.getSecond(), all_edge);

			//Make a string out of the order of vertex.	
			std::string string = gp_model.makeString(all_edge);
			
			std::deque< graphmatch::Match > out;
			std::string operation;
			int dist_temp = gp.editDistance(the_pair.getFirst(), string, all_edge, out, operation);
			dist = dist + dist_temp;
			
// 			drawHypo(gp, gp_model, hypothesis.getMatches(), "hypo");
// 			drawHypo(gp, gp_model, out, "Match");
// 			std::deque < graphmatch::Match> wroking;
// 			wroking.push_back(the_pair);
// 			drawHypo(gp, gp_model, wroking, "wroking on");
// 			cv::waitKey(0);
			
			bool flag_sub = false;
			for(size_t i = 0 ; i < operation.size(); i++){
				if(operation.at(i) == 's'){
					flag_sub = true;
				}
			}
			
			//For the hypothesis of there is a substitution of nodes ie room became corner or recipro
			//Thus we fork the hypo here to analyse the case where the substituion as or hasn't be made in the clustering step
			
// 			if(flag_sub == true && hypothesis.size() >= _min_size_hypo_for_cluster){
			if(flag_sub == true){
				hypothesis.setDist(dist);
				_hypothesis_to_cluster.push_back( hypothesis );
			}
			
			//Add new substitution (ie which node is comaprable to which node in the other graph, ie all new good match) occuring to Q
			for(size_t i = 0 ; i < out.size() ; i++){
				
				//Test if no new vertex as been seen before in this hypotheses
				if(seen_before.isSeen(out[i]) == false){
					Q.push_back(out[i]);
					seen_before.push_back(out[i]);
				}
				
				
				int index;
				//if the pair to match is part of the hypothese, we remove it from the pair to explore later :). This speed up the process and remove redundancy of hypotheses
				if(starting_seeds.isSeen(out[i], index) == true){
					starting_seeds.erase(starting_seeds.begin() + index);
				}
			}
		
		// If Q is not empty : go to fetch next substitute
		}
		
		//Add biggest final hypothese to the hypothesis to tried in clustering.
		if(hypothesis.size() >= _min_size_hypo_for_cluster){
			hypothesis.setDist(dist);
			_hypothesis_to_cluster.push_back( hypothesis );
		}
		
		//Add the final biggest hypo to the possible set of solution
		int newdist = hypothesis.updateDistance(gp, gp_model);
		hypothesis.setDist(newdist);
// 		std::cout << "hypo size "  << hypothesis.size()<< " dist : " << newdist << std::endl;
// 		
// 		drawHypo(gp, gp_model, hypothesis.getMatches(), "inside");
// 		cv::waitKey(0);
		
		_hypothesis_final.push_back(hypothesis);
		
// 		drawHypo(gp, gp_model, _hypothesis_final[0].getMatches(), "inside in deque");
// 		cv::waitKey(0);
		
		hypothesis.clear();
		
	}
	
	//Cluster the hypothesis to cluster
	if(_hypothesis_to_cluster.size() == 0 && _hypothesis_final.size() == 0){
		return false;
	}
	else if(_hypothesis_to_cluster.size() == 0 && _hypothesis_final.size() != 0){
		return true;
	}
	else{
		std::cout << "Clustered !!!!!!!!!!!!!!!!!!!!" << std::endl;
		cluster(_hypothesis_to_cluster, gp, gp_model);
		return true;
	}
	
}


void AASS::graphmatch::GraphMatcherClusterFiltered::cluster(
	std::deque < graphmatch::Hypothese > & hypothesis_to_test, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
{
	
	//First sort the hypo to test so we are sure the biggest ones get tested wth the small ones and we are not passing by small ones that we erase later.
	
	std::cout << "Size of hypo in lcuster " << hypothesis_to_test.size() << std::endl;
	
	//TODO NEW SORT BASED ON SIZE OF HYPO
	sort_by_size(hypothesis_to_test);
	
	std::deque < graphmatch::Cluster > deque_cluster;
	std::deque < graphmatch::Hypothese >::iterator it_hypo;
	
	//For each hypothesis
// 		for(it_hypo = hypothesis_to_test.begin() ; it_hypo != hypothesis_to_test.end()-1; it_hypo++){
	int count = 0 ;
	while( hypothesis_to_test.size() != 0 ){
		
		if(hypothesis_to_test.size() % 100 == 0){	
			std::cout << "hypothesis_to_test size : " << hypothesis_to_test.size() << std::endl;
		}
		if(count % 100 == 0 ){
			std::cout << "100 turns in clustering : " << hypothesis_to_test.size() << std::endl;
		}
		
		graphmatch::Cluster cluster;
		cluster.push_back(hypothesis_to_test[0]);
		
		//For each other hypothesis later to test
		for(size_t j = 1 ; j < hypothesis_to_test.size() ; j++){
			
// 			std::cout << "hypothesis_to_test size : " << hypothesis_to_test.size() << std::endl;
// 			drawHypo(gp, gp_model, hypothesis_to_test[j].getMatches(), "Test with");
// 			drawHypo(gp, gp_model, hypothesis_to_test[0].getMatches(), "compatible with");
// 			drawHypo(gp, gp_model, hypothesis_to_test[j].getMatches(), "Test with");
// 			drawHypo(gp, gp_model, hypothesis_to_test[0].getMatches(), "compatible with");
			if( cluster.isCompatible(hypothesis_to_test[j], gp, gp_model) == true){
	
				
				//To much strain here need to be lighter.
// 				graphmatch::Hypothese hyphyp;
// 				hyphyp.fuse(hypothesis_to_test[j]);
// 				hyphyp.fuse(cluster.getCluster());
// 				hypothesis_to_test.push_back(hyphyp);
				std::cout << "Good " << std::endl;
				//Put the hypothesis in the cluster of hypothesis
				cluster.push_back(hypothesis_to_test[j]);
				
// 				drawHypo(gp, gp_model, hyphyp.getMatches(), "result comparison");
				
			}
// 			cv::waitKey(0);
		}
		deque_cluster.push_back(cluster);
		graphmatch::Hypothese hypo_final;
		//Put every cluster as an hypo
		for(size_t i = 0 ; i < cluster.size() ; i ++){
			for(size_t u = 0 ; u < cluster[i].size() ; u++){
				hypo_final.push_back(cluster[i][u]);
			}
		}
		
		//Calculate new cost
// 		drawHypo(gp, gp_model, hypo_final.getMatches(), "test");
		
		int cost = hypo_final.updateDistance(gp, gp_model);
		
		std::cout << "end of cluser wiht dist" << cost << std::endl;
		//Add it hypo
		hypo_final.setDist(cost);
		_hypothesis_final.push_back( hypo_final );
		
		hypothesis_to_test.pop_front();
		count++;
		
	}
	
	
}
