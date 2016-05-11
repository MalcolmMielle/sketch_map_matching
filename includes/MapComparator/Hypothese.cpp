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
		
		std::string str = gp[_hypothesis[i].getFirst()].getID();
		std::string str2 = gp_model[_hypothesis[i].getSecond()].getID();
		
// 		std::cout << "types " << str << " " << str2 << std::endl;
		
		//Add difference of type
		if(str != str2){
			sub++;
		}
		
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