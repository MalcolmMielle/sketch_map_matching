#include "Hypothese.hpp"

//TODO : update the distance manually. SHOULD NOT BE USED. USe normalized Edt Distance
int AASS::graphmatch::Hypothese::updateDistance(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model) const
{
	
// 	std::cout << "updating distance fror hypo of size " << size() << std::endl;
	gp.resetLabel();
	gp_model.resetLabel();
	
	StorageFilter sf;
	StorageFilter sf_model;
	boost::keep_all _keep_all;
	graphmatch::Filtered_place fp(gp.getGraph(), _keep_all, boost::bind(&StorageFilter::removedVertices, &sf, _1));
	graphmatch::Filtered_place fp_model(gp_model.getGraph(), _keep_all, boost::bind(&StorageFilter::removedVertices, &sf_model, _1));		
	

	/*
		* For every non matched VertexPlace
		* 		remove Edge still there
		* 		remove Vertex
		* 		update distance
		* big = biggest number of edge between fp and fp_model
		* small = ...
		* dist = dist + (big - small)
	*/
	
	int sub = 0 ;
	std::deque< graphmatch::VertexPlace> dvp;
	std::deque< graphmatch::VertexPlace> dvp_model;
	for(size_t i = 0 ; i < _hypothesis.size() ; i++){
		
		dvp.push_back(_hypothesis[i].getFirst());
		dvp_model.push_back(_hypothesis[i].getSecond());
		
		//TODO THIS FUNCTION SHOULD NEVER BE USED
// 		bool isR = gp.isRoom(_hypothesis[i].getFirst());
// 		bool isR_model = gp_model.isRoom(_hypothesis[i].getSecond());
		
// 		std::string str = gp[_hypothesis[i].getFirst()].getID();
// 		std::string str2 = gp_model[_hypothesis[i].getSecond()].getID();
		
		bool same = gp[_hypothesis[i].getFirst()].compare(gp_model[_hypothesis[i].getSecond()]);
		
// 		std::cout << "types " << str << " " << str2 << " and same is " << same << std::endl;
		
// 		if(str == str2){
// 			assert(same == true);
// 		}
// 		else{
// 			assert(same == false);
// 		}
		//Add difference of type
		if(same == true){
			std::cout << "sub+ " ;
			
		}
// 		if(str == str2){
// 			sub++;
// 			std::cout << "sub+" << std::endl;
// 		}
		
	}
	
// 	std::cout << "There is " << sub << " different room/corner " << std::endl;
			
	gp.labelAll(dvp);
	gp_model.labelAll(dvp_model);
			
	int nb_edges = 0 ;
	int nb_ver = 0;
	
	std::pair<graphmatch::VertexIteratorPlaceFiltered, graphmatch::VertexIteratorPlaceFiltered> vp;
	for (vp = boost::vertices(fp); vp.first != vp.second; ) {
		graphmatch::VertexPlace v = *vp.first;
		if(fp[v].label == false){
			nb_edges = nb_edges + boost::out_degree(v, fp);
			nb_ver++;
			++vp.first;
			sf.addToRemove(v);
		}
		
		else{
			++vp.first;
		}
		
	}
// 	std::cout << "all the numbers edge and vert to remove in model" <<nb_edges <<" " <<nb_ver<<" "<< std::endl;
// 	std::cout << "FULL SIZE " << gp.getNumVertices() << std::endl;
// 	std::cout << "FULL SIZE " << gp_model.getNumVertices() << std::endl;
	
	std::pair<graphmatch::VertexIteratorPlaceFiltered, graphmatch::VertexIteratorPlaceFiltered> vp_model;
	for (vp_model = boost::vertices(fp_model); vp_model.first != vp_model.second; ) {
		graphmatch::VertexPlace v_model = *vp_model.first;
		if(fp_model[v_model].label == false){
			nb_edges = nb_edges + boost::out_degree(v_model, fp_model);
			nb_ver++;
			++vp_model.first;
			sf_model.addToRemove(v_model);
		}
		
		else{
			++vp_model.first;
		}
		
	}
// 	std::cout << "all the numbers edge and vert to remove in input AND model " <<nb_edges <<" " <<nb_ver << std::endl;
	
	/* I don't know of a simple way to retrieve the edges in a filtered graph because I'm able to store the romved edges ... :(
		CaLCULATE the number of edge to remove or add in between vertices that should NOT be removed.
		*/
	int big = 0 ;
	int small = 0 ;
	
	//vertices access all the vertix
	for (vp = boost::vertices(fp); vp.first != vp.second; ) {
		graphmatch::VertexPlace v = *vp.first;
		big = big + boost::out_degree(v, fp);
		++vp.first;
	}
	big = big / 2 ; //<- because every edge is seen twice at its start and at its end
		
	//vertices access all the vertix
	for (vp_model = boost::vertices(fp_model); vp_model.first != vp_model.second; ) {
		graphmatch::VertexPlace v_model = *vp_model.first;
		small = small + boost::out_degree(v_model, fp_model);
		++vp_model.first;			
	}
	
	small = small / 2;

	if(big < small){
		int tmp = big;
		big = small;
		small = tmp;
	}
	
// 	std::cout << " should be 0 (?) but difference still " << big - small << " " << sub << std::endl;
	
	gp.resetLabel();
	gp_model.resetLabel();
	
// 	std::cout << "total " << nb_edges + nb_ver + (big - small) + sub << std::endl;
	
	return nb_edges + nb_ver + (big - small) + sub;
	
}



std::tuple<AASS::graphmatch::Match, AASS::graphmatch::Match> AASS::graphmatch::Hypothese::getLinkedMatches(const AASS::graphmatch::Match& match_equivalent) {

	//Find equivalent matches
	Match match_here_out;
//	Match match_equi_out;
	Match other_here_out;
	int index_other = -1;

	for(auto match_here : _hypothesis) {

		if (match_equivalent.getFirst() == match_here.getFirst()) {
			std::cout << "SEEN" << std::endl;
	//					is_in_final = true;
			//Both match to compare
			match_here_out = match_here;
//			match_equi_out = match_equivalent;

			/* We found a common node in the input graph.
			* We need to compare the two match to know if replacing it is an advantage or not
			* But to compare it, maybe, the new match is matched in the model graph to a very good vertex.
			* So we need to compare the new match to the match of the input AND the model in -->final<--.
			* */
			other_here_out = match_here;
	//					index_other = -1;
			for (auto match_final_2 : _hypothesis) {
				//Look if the second vertex is the same AND if the match is not the already selected match
				if (match_equivalent.getSecond() == match_final_2.getSecond() && match_final_2 != match_here) {
	//								std::cout << "Found at " << f << std::endl;
					other_here_out = match_final_2;
	//								index_other = f;
				}
			}
		}//Test the second part
		else if (match_equivalent.getSecond() == match_here.getSecond()) {
			std::cout << "SEEN2" << std::endl;
	//					is_in_final = true;
			//Both match to compare
			match_here_out = match_here;
//			match_equi_out = match_equivalent;

			/* We found a common node in the input graph.
			* We need to compare the two match to know if replacing it is an advantage or not
			* But to compare it, maybe, the new match is matched in the model graph to a very good vertex.
			* So we need to compare the new match to the match of the input AND the model in -->final<--.
			* */
			other_here_out = match_here;
			//					index_other = -1;
			for (auto match_final_2 : _hypothesis) {
				//Look if the second vertex is the same AND if the match is not the already selected match
				if (match_equivalent.getFirst() == match_final_2.getFirst() && match_final_2 != match_here) {
					//								std::cout << "Found at " << f << std::endl;
					other_here_out = match_final_2;
					//								index_other = f;
				}
			}
		}
	}

	return std::make_tuple(match_here_out, other_here_out);
}


bool AASS::graphmatch::Hypothese::shouldReplaceBasedOnCost(const AASS::graphmatch::Match& match_equivalent) {
	auto [match_final_to_compare, other_final_to_compare] = getLinkedMatches(match_equivalent);
	size_t diff = 0;
	bool better = match_final_to_compare.bestMatch(match_equivalent, other_final_to_compare, diff);
// 	std::cout << "COST : " << better << std::endl;
	//The new match got more confidence than both others.
	std::cout << "DIFFERENCE " << diff << std::endl;

	return better;


}