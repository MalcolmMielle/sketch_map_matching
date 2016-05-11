#include "ProbabilisticEditDistanceEdge.hpp"

//ATTENTION FULL WRONG FOR NOW
bool AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::match(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model)
{
	
	/*Init
	
	Create all probabilities matching <- Done
	
	Get anchors <- Done
	
	Get cost of full matching <- Done
	
	while
		find more anchors with proba
		match planar
		get cost
	*/
	
	init(gp, gp_model);
	
	std::deque < graphmatch::Match > anchor;
	getAnchorsInit(gp, gp_model, anchor);
	
	//No doors in the graphs
// 	if(anchor.size() == 0){
// 		getAnchors(gp, gp_model, anchor);
// 	}
	
	_anchor_matcher.anchorMatching(gp, gp_model, anchor);
	
	graphmatch::Hypothese res_ante = _anchor_matcher.getResult()[0];
	int cost_ante = res_ante.getDist();
	
	//First matching with anchor
	getAnchors(gp, gp_model, anchor);
	//Calculate new cost
	_anchor_matcher.clear();
	_anchor_matcher.anchorMatching(gp, gp_model, anchor);
	
	graphmatch::Hypothese res = _anchor_matcher.getResult()[0];
	int cost = res.getDist();
	
	while (cost_ante > cost){
		cost_ante = cost;
		res_ante = res;
		
		getAnchors(gp, gp_model, anchor);
		//Calculate new cost
		_anchor_matcher.clear();
		_anchor_matcher.anchorMatching(gp, gp_model, anchor);
		graphmatch::Hypothese res = _anchor_matcher.getResult()[0];
		cost = res.getDist();
	}
	
	_hypothesis_final.push_back(res_ante);

	return false;
}


void AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::getAnchorsInit(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp2, std::deque < graphmatch::Match >& anchor)
{
	
	//Find doors 
	
	std::deque < graphmatch::VertexPlace > doors;
	std::deque < graphmatch::VertexPlace > doors_model;
	
	getDoors(doors, gp);
	getDoors(doors_model, gp2);
		
	//Use probabilities on the doors
	std::deque < int > alreadyUsed;
	for(size_t i = 0 ; i < doors.size() ; i++){
		
		double proba = 0 ;
		//Create match base of the door
		graphmatch::Match pair(doors[i], doors_model[0]) ;
		pair.setProba(proba);
		//Id of the mtahc door
		int id = -1;
		
		for(size_t j = 0 ; j < doors_model.size() ; j++){
			
			double prob_tmp = _graphs.getPUUFF(pair.getFirst(), doors_model[j] ) ;
			
			if(proba <= prob_tmp ){
				
				bool seen = false;
				//Make sure the door was not already used.
				for(size_t z = 0 ; z < alreadyUsed.size() ; z++){
					if( (int) j == alreadyUsed[z]){
						seen = true;
					}
				} 
				
				//Create the anchor if all condition are there.
				if(seen == false){
					pair.setSecond(doors_model[j]);
					pair.setProba(prob_tmp);
					id = j;
				}
			}
			
		}
		
		//Add the anchor if a new one as been created.
		if(id != -1){
			anchor.push_back(pair);
			alreadyUsed.push_back(id);
		}
		
	}
	

}

//Stupid since the door don't have the best proba for sure. ATTENTION : SUPER WRONG
void AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::getAnchors(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp2, std::deque < graphmatch::Match >& anchor)
{
	
	double min_proba = 1;
	double max_proba = 0;
	for(size_t i= 0 ; i < anchor.size() ; i++){
		if (min_proba > anchor[i].getProba()){
			min_proba = anchor[i].getProba();
		}
		if (max_proba < anchor[i].getProba()){
			max_proba = anchor[i].getProba();
		}
	}
	
	size_t j = 0 ;
	graphmatch::Match new_match(_graphs.getMatch(j));
	j++;
	bool good = false;
	while( good == false && j < _graphs.size() ){
		
		new_match = _graphs.getMatch(j);
		
		//good proba
		std::cout << std::endl << "New trying" << std::endl ;
		if( new_match.getProba() <= min_proba || new_match.getProba() >= max_proba ) {
			//Not already seen
			bool seen = false;
			for(size_t c = 0 ; c < anchor.size() ; c++){
				//Already seen
				if(new_match.sameVertexThan ( anchor[c] ) == true){
					seen = true;
				}
			}
			if( seen == false ){
// 				std::cout << "that was false" << std::endl;
				good = true;
			}
		}
		j++;
	}
	if(j == _graphs.size()){
		throw std::runtime_error("No new anchors to find");
	}
	else{
		anchor.push_back(new_match);
	}

}


void AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::getDoors(std::deque< AASS::graphmatch::VertexPlace >& doors, AASS::graphmatch::GraphPlace& gp)
{

	std::pair<graphmatch::VertexIteratorPlace, graphmatch::VertexIteratorPlace> vp;
	//vertices access all the vertix
	for (vp = boost::vertices( gp.getGraph() ); vp.first != vp.second; ++vp.first) {
		graphmatch::VertexPlace v = *vp.first;
		
		std::string type = gp[v].getID();
		if( type == "d"){
			doors.push_back(v);
		}
		
	}
}


void AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::init(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model)
{
	_hypothesis_final.clear();
	_graphs.setProba(gp, gp_model);
	_graphs.setInput(gp);
	_graphs.setModel(gp_model);
	_graphs.sort();
}



bool AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::algo4(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model) 
{
// 	std::cout << "Init" << std::endl;
	init(gp, gp_model);
// 	std::cout << "Done init" << std::endl;
	graphmatch::Hypothese anchor;
	
	#ifdef TIMED
	timed("anchorInit", boost::bind( &ProbabilisticEditDistanceEdge::getAnchorsInit, this, boost::ref(gp), boost::ref(gp_model), boost::ref(anchor) ) );
	#else
	getAnchorsInit(gp, gp_model, anchor.getMatches());	
	#endif
	
// 	getAnchorsInit(gp, gp_model, anchor);	
// 	_anchor_matcher.anchorMatching < MatchProb > (gp, gp_model, anchor);
	
// 	graphmatch::Hypothese res_ante = _anchor_matcher.getResult()[0];
// 	int cost = res_ante.getDist();
	graphmatch::Hypothese res_ante;
	int cost = res_ante.updateDistance(gp, gp_model);
	
// 	std::cout << "Cost at sta rt " << cost << std::endl;
	cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");
// 	printProbaGraph(_graphs, mat_in, mat_in);
	
	bool flag = true;
	while(flag == true){
// 		std::cout << "Loop in algo 4. anchor size : " << anchor.size() << std::endl;
		//Return all the best anchors
		
		std::deque < graphmatch::Match > temp_anchor;
		getAllBestAnchors(gp, gp_model, anchor.getMatches(), temp_anchor);
		//If there is a new anchor
		
		bool new_good_anchor = false;
		
// 		_graphs.print();
					
// 		std::cout << "Temp anchor size : " << temp_anchor.size() << std::endl;
// 		for(size_t i = 0 ; i < temp_anchor.size() ; i++){
// 			std::cout << "proiba : " << temp_anchor[i].getProba() << std::endl;
// 			cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");
// 			drawHypo<MatchProb>(gp, gp_model, mat_in, mat_in, temp_anchor, "ANCHORS", 2);
// 			cv::waitKey(0);
// 		}
		
// 		int a ;
// 		std::cout << "Pause before testing " << temp_anchor.size() << " with cost " << cost << std::endl;
// 		std::cin >> a;
		
		if(temp_anchor.size() > 0){
			int best_cost = - 1;	
			graphmatch::Match best_match;
			//For all temporary anchor
			for(size_t i = 0 ; i < temp_anchor.size() ; ++i){
				std::cout << "test number : " << i << " size of anchor temp " << temp_anchor.size() << std::endl;
				
				//Get new match
// 				best_match = temp_anchor[i];
				anchor.push_back(temp_anchor[i]);
				
				anchor.drawHypo(gp, gp_model, mat_in, mat_in, "Anchor", 2);
				
				_anchor_matcher.clear();
				
				//Calculate new planar matching with added anchor
// 				std::clock_t a = std::clock();
// 				std::cout << "ANCHOR size : " << anchor.size() << std::endl;
 				
				
#ifdef TIMED
				std::ostringstream str_test;
				str_test <<  "anchor matching, line" << __LINE__ << " in file " << __FILE__;
				timed(str_test.str(), boost::bind( &graphmatch::GraphMatcherAnchor::anchorMatching, &_anchor_matcher, boost::ref(gp), boost::ref(gp_model), boost::ref(anchor.getMatches()) ) );
#else
				_anchor_matcher.anchorMatching(gp, gp_model, anchor.getMatches());
#endif
// 				
// 				std::cout << "ANCHOR DONE : " <<_anchor_matcher.getResult().size() << std::endl;
// 				std::clock_t b = std::clock();
// 				std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< std::endl;

				graphmatch::Hypothese res_ante_temp = _anchor_matcher.getResult()[0];
				int cost_temp = res_ante_temp.getDist();
				
				//Saves the best cost in all the test
				
				//Remove the anchor we just tested
				
				//Weird result there
				
// 				std::cout << "COST : " << cost_temp << " PREVIOUS COST " << cost << " BEST COST " << best_cost << std::endl;
// 				mat_in = cv::imread("../Test/Sequences/missingmap.png");
// 				drawHypo(gp, gp_model, mat_in, mat_in, anchor, "Anchor", 2);
// 				drawMoved(gp, gp_model, mat_in, mat_in, res_ante_temp.getMatches(), "Fusiond", 2);
// 				cv::waitKey(0);
				
// 				containsDoubleMatch();
				if(anchor.size() > 1 && i > 636){
					std::cout << "Drawing now " << std::endl;
					mat_in = cv::imread("../Test/Sequences/missingmap.png");
					anchor.drawHypo(gp, gp_model, mat_in, mat_in, "Anchor", 2);
					res_ante_temp.drawHypo(gp, gp_model, mat_in, mat_in, "Anchor fusiond", 2);
// 					drawMoved(gp, gp_model, mat_in, mat_in, res_ante_temp.getMatches(), "Fusiond", 2);
					
					graphmatch::GraphMatcherAnchor gma_test;
					gma_test.anchorMatching(gp, gp_model, anchor.getMatches(), true);
					
					std::cout << std::endl << " Before WAS FUSION " << std::endl << std::endl;

					
					std::deque < graphmatch::Match > anchor_testy;
					anchor_testy.push_back(anchor[0]);
					gma_test.clear();
					gma_test.anchorMatching(gp, gp_model, anchor_testy);
					graphmatch::Hypothese res_ante_temp_testy = gma_test.getResult()[0];
					res_ante_temp_testy.drawHypo(gp, gp_model, mat_in, mat_in, "Fusion first match", 2);
					
					std::cout << "Cost " << res_ante_temp_testy.getDist()  << " " << cost << std::endl;
					
					gma_test.clear();
					
					anchor_testy.clear();
					anchor_testy.push_back(anchor[1]);
					gma_test.clear();
					gma_test.anchorMatching(gp, gp_model, anchor_testy);
					res_ante_temp_testy = gma_test.getResult()[0];
					res_ante_temp_testy.drawHypo(gp, gp_model, mat_in, mat_in, "Fusion second match", 2);
					
					std::cout << std::endl << " BEST FOR NOW " << std::endl << std::endl;
					anchor_testy.clear();
					anchor_testy.push_back(anchor[0]);
					anchor_testy.push_back(best_match);
					gma_test.clear();
					gma_test.anchorMatching(gp, gp_model, anchor_testy);
					res_ante_temp_testy = gma_test.getResult()[0];
					res_ante_temp_testy.drawHypo(gp, gp_model, mat_in, mat_in, "Best of the best for now", 2);
					
					std::cout << "Cost " << res_ante_temp_testy.getDist()  << cost << std::endl;
					
					std::cout << "Cost studied one " << cost_temp << " best " << best_cost << " cost before" << cost << std::endl;
					cv::waitKey(0);
				}
				
				
				
				//If the score are better than everything before
// 				std::cout << "the costs. costtemp "<< cost_temp << " cost " << cost << " best cost " <<  best_cost << std::endl;
				if(cost_temp < cost ){
					
					
					if(best_cost == -1 || cost_temp < best_cost){
// 						std::cout << "Best :D : " << cost_temp << std::endl;
						
						
						//Update best cost
						best_cost = cost_temp;
						//Update best anchor
						best_match = temp_anchor[i];
						//Flag the finding of a new anchor
						new_good_anchor = true;
						
						
						
						
// 						graphmatch::GraphMatcherAnchor gma_test;
// 						
// 						std::cout << std::endl << " Before WAS FUSION " << std::endl << std::endl;
// 
// 						
// 						std::deque < graphmatch::Match > anchor_testy;
// 						
// 						anchor_testy.push_back(anchor[0]);
// 						anchor_testy.push_back(best_match);
// 						gma_test.clear();
// 						gma_test.anchorMatching(gp, gp_model, anchor_testy);
// 						graphmatch::Hypothese res_ante_temp_testy = gma_test.getResult()[0];
// 						drawHypo(gp, gp_model, mat_in, mat_in, res_ante_temp_testy.getMatches(), "YEAH", 2);
// 						drawHypo(gp, gp_model, mat_in, mat_in, anchor_testy, "YEAH acnhor", 2);
// 						
// 						std::cout << "Cost " << res_ante_temp_testy.getDist()  << cost << std::endl;
// 						
// 						std::cout << "Cost od studied one " << cost_temp << " best " << best_cost << " cost before" << cost << std::endl;
// 						cv::waitKey(0);

						
						
						
					}
				}
				
				anchor.pop_back();
				
			}
			//If a good new anchor was found, update all variables
			if(new_good_anchor == true){
				
				//Save the best anchor
				anchor.push_back(best_match);
				
// 				cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");
// 				drawHypo(gp, gp_model, mat_in, mat_in, anchor, "Anchor after add", 2);
				
				/********** TEST ONLY**************/
				
				_anchor_matcher.clear();
				
// 				std::cout << "ANCHOR size : " << anchor.size() << std::endl;
				#ifdef TIMED
				std::ostringstream str;
				str << "anchor matching, line" << __LINE__ << " in file " << __FILE__;
				timed(str.str(), boost::bind( &graphmatch::GraphMatcherAnchor::anchorMatching, &_anchor_matcher, boost::ref(gp), boost::ref(gp_model), boost::ref(anchor.getMatches()) ) );
				#else
				_anchor_matcher.anchorMatching(gp, gp_model, anchor.getMatches());
				#endif
				
// 				std::cout << "ANCHOR DONE : " <<_anchor_matcher.getResult().size() << std::endl;
// 				std::clock_t b = std::clock();
// 				std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< std::endl;

// 				graphmatch::Hypothese res_ante_temp = _anchor_matcher.getResult()[0];
// 				drawMoved(gp, gp_model, mat_in, mat_in, res_ante_temp.getMatches(), "Fusion Moved", 2);
// 				std::cout << "COST " << res_ante_temp.getDist() << std::endl;
				
				/******************************/
				
// 				cv::waitKey(0);
				
				//Update cost
				cost = best_cost;
			}
			else{
				flag = false;
			}
			
		}
		else{
// 			std::cout << "Flag set to false" << std::endl;
			flag = false;
		}
				
	}
	
	if(anchor.size() == 0 ){
		throw std::runtime_error("No anchor found and that's weird. It means no match is better than using any anchor");
	}
	
	_anchor_matcher.clear();
	bool res_bool = _anchor_matcher.anchorMatching(gp, gp_model, anchor.getMatches());
	graphmatch::Hypothese res = _anchor_matcher.getResult()[0];
	cost = res.getDist();
	_hypothesis_final.push_back(res);
// 	std::cout << "END OF ALGO 4 " << std::endl;
	return res_bool;

}


//TODO : optimize this. We consider _graphs sorted.
void AASS::probabilisticmatching::ProbabilisticEditDistanceEdge::getAllBestAnchors(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp2, const std::deque< AASS::graphmatch::Match >& anchor, std::deque< AASS::graphmatch::Match >& temp_anchor)
{
	
	
	graphmatch::Match new_match;
	graphmatch::Match best_match;
	
	double prob_tmp = -1;
	//Get the new best cobination
	int place = 0;
	for(size_t j = 0 ; j < _graphs.size() && prob_tmp == -1 ; j++){
		
		new_match = _graphs.getMatch(j);
		
		//Not already seen
		bool seen = false;
		for(size_t c = 0 ; c < anchor.size() ; c++){
			//Already seen
			if(new_match.sameVertexThan ( anchor[c] ) == true){
				seen = true;
			}
		}
		if( seen == false ){
			prob_tmp = new_match.getProba();
			temp_anchor.push_back(new_match);
			place = j;
		}
	}
	
// 	std::cout << "Found a new anchor " << temp_anchor.size() << std::endl;
	
	//If nothing was found
	if(temp_anchor.size() != 0){
		//Get all the same value match
		bool tobreak = false;
		for(size_t j = place + 1 ; j < _graphs.size() && tobreak == false ; j++){
			new_match = _graphs.getMatch(j);
// 			std::cout << "Comparaison at " << j << " " << temp_anchor[0].getProba() << " and " << new_match.getProba() << std::endl;

			//Not already seen
			bool seen = false;
			for(size_t c = 0 ; c < anchor.size() ; c++){
				//Already seen
				if(new_match.sameVertexThan ( anchor[c] ) == true){
					seen = true;
				}
			}
			
			//Break if less
			if(temp_anchor[0].getProba() > new_match.getProba()){
				tobreak = true;
			}
			else if( seen == false && new_match.getProba() == temp_anchor[0].getProba() ){
				temp_anchor.push_back(new_match);
			}
		}
	}
	
}




