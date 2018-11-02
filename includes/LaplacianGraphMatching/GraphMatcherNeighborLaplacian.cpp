#include "GraphMatcherNeighborLaplacian.hpp"


int AASS::graphmatch::GraphMatcherNeighborLaplacian::initPlanar(const AASS::graphmatch::MatchLaplacian& the_pair, AASS::graphmatch::HypotheseLaplacian& starting_seeds, AASS::graphmatch::HypotheseLaplacian& Q, AASS::graphmatch::HypotheseLaplacian& seen_before, graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model)
{
	//Match neighborhoods
// 			std::cout << "Getting edges" << std::endl;
	std::deque< std::pair< graphmatch::GraphLaplacian::EdgeLaplacian, graphmatch::GraphLaplacian::VertexLaplacian > > all_edge;
	gp_model.getAllEdgeLinkedCounterClockWise(the_pair.getSecond(), all_edge);

	//Make a string out of the order of vertex.
// 			std::string string = gp_model.makeString(all_edge);
// 			std::cout << "result : "<< string;
	std::deque< graphmatch::MatchLaplacian > out;
	std::string operation;

	/*
	 * Calculate the edit distance between every string out of the first pair and string
	 */

	std::deque< Region> out_places;
	gp_model.getAllVertexAttrCounterClockWise(the_pair.getSecond(), out_places);

	int dist_temp = gp.editDistance(the_pair.getFirst(), out_places, all_edge, out, operation);

// 			int dist_temp = gp.editDistance(the_pair.getFirst(), string, all_edge, out, operation);
// 			std::cout << "Result : " << operation << std::endl;
	//Add new substitution (ie which node is comaprable to which node in the other graph, ie all new good match) occuring to Q
	for(size_t i = 0 ; i < out.size() ; i++){

		Q.push_back(out[i]);
		seen_before.push_back(out[i]);

		int index_out;
		//if the pair to match is part of the hypothese, we remove it from the pair to explore later :). This speed up the process and remove redundancy of hypotheses
		if(starting_seeds.isSeen(out[i], index_out) ){
			starting_seeds.erase(starting_seeds.begin() + index_out);
		}
	}

	return dist_temp;
}


bool AASS::graphmatch::GraphMatcherNeighborLaplacian::planarEditDistanceAlgorithm(graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model)
{

	//Determine seed substitutions (ie which node is comaprable to which node in the other graph)
	//Add seed substitution to the FIFO queue Q

	graphmatch::HypotheseLaplacian starting_seeds;
	gp.pairWiseMatch(gp_model, starting_seeds.getMatches());

// 	cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");
// 	drawHypoSlow(gp, gp_model, mat_in, mat_in, starting_seeds.getMatches(), "Starting Seeds", 2);

	bool res = planarEditDistanceAlgorithm(starting_seeds, gp, gp_model);
	return res;

}


bool AASS::graphmatch::GraphMatcherNeighborLaplacian::planarEditDistanceAlgorithm(graphmatch::HypotheseLaplacian& starting_seeds, graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp_model)
{

	graphmatch::HypotheseLaplacian seen_before;
	graphmatch::HypotheseLaplacian Q;
	graphmatch::HypotheseLaplacian hypothesis;
	std::cout << "Size of pair wise " << starting_seeds.size() << std::endl;

	//ATTENTION : Suppressed this for to cluster because it become really slow.
	for(size_t i = 0 ; i < starting_seeds.size() ; i++){
		graphmatch::HypotheseLaplacian hyp;
		hyp.push_back(starting_seeds[i]);
		int newdist = hyp.updateDistance(gp, gp_model);
		hyp.setDist(newdist);
		_hypothesis_final.push_back(hyp);
	}
	//To keep all hypo by themselves

	while(starting_seeds.empty() == false){

// 		cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");

		seen_before.clear();
		Q.push_back(starting_seeds.at(0));
// 		drawHypo(gp, gp_model, mat_in, mat_in, Q.getMatches(), "the seed", 2);

		seen_before.push_back(starting_seeds.at(0));
		starting_seeds.pop_front();

		graphmatch::MatchLaplacian the_pair_init(Q[0].getFirst(), Q[0].getSecond());

		Q.pop_front();
		hypothesis.push_back(the_pair_init);
// 		std::cout << "Init" << std::endl;
		double dist = initPlanar(the_pair_init, starting_seeds, Q, seen_before, gp, gp_model);



// 		cv::Mat mat_in = cv::imread("../Test/FromVirtual/Ravi/ObstacleMap.png");
// 		cv::Mat model = cv::imread("../Test/FromVirtual/map_2.png");
// 		drawHypo(gp, gp_model, mat_in, model, hypothesis.getMatches(), "INIT");
// 		drawHypo(gp, gp_model, mat_in, model, Q.getMatches(), "Match");
// 		std::cout << "INIT" << std::endl;
// 		cv::waitKey(0);

// 		std::cout << "End of init" << std::endl;
		while(Q.empty() == false){

// 			int newdistt = hypothesis.updateDistance(gp, gp_model);
// 			std::cout << "hypo size "  << hypothesis.size()<< " dist : " << newdistt << std::endl;

// 				std::cout << " size of Q : " << Q.size() << std::endl;

			//Fetch next substitution from Q
			graphmatch::MatchLaplacian the_pair(Q[0]);

			Q.pop_front();
			hypothesis.push_back(the_pair);

// 			//Match neighborhoods
// 			std::cout << "Going to make the match" << std::endl;

			std::deque< graphmatch::MatchLaplacian > out_match;
			dist = dist + gp.makeMatching(the_pair.getFirst(),
			                                       the_pair.getSecond(),
			                                       gp_model,
			                                       hypothesis.getMatches(),
			                                       out_match);


			//Add the edit distance to all the match
			for(size_t i = 0 ; i < out_match.size() ; i++){
				//Set the cost as the normalized edit distance
				out_match[i].setCost(dist);
// 				std::cout << "DIst : " << out_match[i].getCost() << std::endl;
			}

// 			std::cout << "SIZE OF MATCHES AT THE END : " << out_match.size() << std::endl;



// 			drawHypo(gp, gp_model, mat_in, model, hypothesis.getMatches(), "hypo");
// 			drawHypo(gp, gp_model, mat_in, model, out_match, "Match");
// 			std::deque < graphmatch::Match> wroking;
// 			wroking.push_back(the_pair);
// 			drawHypo(gp, gp_model, mat_in, model, wroking, "wroking on");
// 			std::cout << "Match size :" << out_match.size() << std::endl;
// 			cv::waitKey(0);


			//Add new substitution (ie which node is comaprable to which node in the other graph, ie all new good match) occuring to Q
			for(size_t i = 0 ; i < out_match.size() ; i++){

				//Test if no new vertex as been seen before in this hypotheses
				if(seen_before.isSeen(out_match[i]) == false){
					Q.push_back(out_match[i]);
					seen_before.push_back(out_match[i]);
				}


				int index_out;
				//if the pair to match is part of the hypothese, we remove it from the pair to explore later :). This speed up the process and remove redundancy of hypotheses
				if(starting_seeds.isSeen(out_match[i], index_out) == true){

// 					std::deque < Match > dm;
// 					dm.push_back(starting_seeds[index_out]);
// 					drawHypo(gp, gp_model, mat_in, mat_in, dm, "removing", 2);
// 					cv::waitKey(0);
					starting_seeds.erase(starting_seeds.begin() + index_out);
				}
			}

			// If Q is not empty : go to fetch next substitute
		}


		//Add the final biggest hypo to the possible set of solution
		int newdist = hypothesis.updateDistance(gp, gp_model);
		hypothesis.setDist(newdist);
// 		std::cout << "hypo size "  << hypothesis.size()<< " dist : " << newdist << std::endl;
//
// 		drawHypo(gp, gp_model, mat_in, mat_in, hypothesis.getMatches(), "inside", 2);
// 		cv::waitKey(0);
// 		for(size_t i = 0 ; i < hypothesis.size() ; i++){
//
// 			std::cout << "              Cost hyp : " << hypothesis[i].getCost() << std::endl;
// 		}
		_hypothesis_final.push_back(hypothesis);

// 		drawHypo(gp, gp_model, _hypothesis_final[0].getMatches(), "inside in deque");
// 		cv::waitKey(0);

		hypothesis.clear();



	}


// 	std::cout << "SIZE OF FINAL AT THE END " << _hypothesis_final.size() << std::endl;
// 	cv::Mat mat_in = cv::imread("../Test/FromVirtual/Ravi/ObstacleMap.png");
// 	cv::Mat model = cv::imread("../Test/FromVirtual/map_2.png");
// 	drawHypo(gp, gp_model, mat_in, model, _hypothesis_final[0].getMatches(), "AND OF PLANAR");
// 	cv::waitKey(0);

	if(_hypothesis_final.size() > 0){
		return true;
	}
	else{
		return false;
	}
}
