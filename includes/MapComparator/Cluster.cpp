#include "Cluster.hpp"

bool AASS::graphmatch::Cluster::isSeen(const Match& pair) const
{

	for(size_t i = 0 ; i < _cluster.size() ; i++){

		for(size_t j = 0 ; j < _cluster[i].size() ; j++){
			if(_cluster[i][j].getFirst() == pair.getFirst() || _cluster[i][j].getSecond() == pair.getSecond()){
				return true;
			}
			else if(_cluster[i][j].getSecond() == pair.getFirst() || _cluster[i][j].getFirst() == pair.getSecond()){
				return true;
			}
		}
	}
	return false;

}



bool AASS::graphmatch::Cluster::isCompatible(const Hypothese h) const
{

	bool compatible = true;
	std::deque< graphmatch::Match > match_h;
	match_h = h.getMatches();

	std::deque< graphmatch::Match >::iterator it;
	for(it = match_h.begin() ; it != match_h.end() ; it++){
		if(isSeen(*it) == true){
			compatible = false;
		}
	}

	return compatible;


}

bool AASS::graphmatch::Cluster::isCompatible(Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
{

	//Basic threshold we try to fuse the two graph only if they are consequent enough.
	int size_cluster = 0;
	for(size_t i = 0 ; i < _cluster.size() ; i++){
		size_cluster = size_cluster + _cluster[i].size();
	}

	size_cluster = size_cluster + hyp_in.size();
	int left_from_graph = gp.getNumVertices() - size_cluster;
	int left_from_graph_model = gp_model.getNumVertices() - size_cluster;

// 	std::cout << "all the num " << gp.getNumVertices() << " " << gp_model.getNumVertices() << " percentage covered " << left_from_graph + left_from_graph_model << " percentage max needed covered " << (gp.getNumVertices() + gp_model.getNumVertices()) * 50 /100 << std::endl;

	if(left_from_graph + left_from_graph_model > (gp.getNumVertices() + gp_model.getNumVertices()) * 25 /100){
// 		std::cout << "Too small" << std::endl;
		return false;
	}


// 		std::cout << "INDISE" << std::endl;
// 		std::cout << hyp_in.size() << std::endl;
	bool compatible = isCompatible(hyp_in);

	if(compatible == false){
		return false;
	}

	//TODO : THIS IS ULTRA DUMB BUT I CAN'T SEEM TO COPY TO FILTEREDGRAPH CLASS
	boost::keep_all _keep_all;
	graphmatch::Filtered_place fp(gp.getGraph(), _keep_all, boost::bind(&StorageFilter::removedVertices, &_sf, _1));
	graphmatch::Filtered_place fp_model(gp_model.getGraph(), _keep_all, boost::bind(&StorageFilter::removedVertices, &_sf_model, _1));

	//Remove all vertex outside of hypotheses
	reduce(hyp_in, gp, gp_model, fp, fp_model);

	graphmatch::Hypothese hyp;
	for(size_t i = 0 ; i < hyp_in.size() ; i++){
		hyp.push_back(hyp_in[i]);
	}

	//Need to push the cluster
	for(size_t i = 0 ; i < _cluster.size() ; i++){
		for(size_t j = 0 ; j < _cluster[i].size() ; j++){
			hyp.push_back(_cluster[i][j]);
		}
	}

	//TODO : make it faster by only considering vertex in between hypothesis and not all vertex.
	compatible = isSame(hyp, gp, fp, gp_model, fp_model);

	//Reset all
	_sf.reset(gp);
	_sf_model.reset(gp_model);
	gp.resetLabel();
	gp_model.resetLabel();

	return compatible;

}



void AASS::graphmatch::Cluster::reduce(graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model, const graphmatch::Filtered_place& fp, const graphmatch::Filtered_place& fp_model)
{

// 		std::deque<VertexPlace> deque_vertex;
	//SUPPRESS ALL VERTEX NOT IN HYPO
	std::deque<VertexPlace> deque_vertex_all;
	std::deque<VertexPlace> deque_vertex_all_model;

	//Get only the first one since it's the reduced hypo
	//Get all vertex in the hypo
	for(size_t j = 0 ; j < hyp_in.size() ; j++){
		deque_vertex_all.push_back(hyp_in[j].getFirst());
	}


	//For all hypothesis in cluster reduce the graph
	for(size_t i = 0 ; i < _cluster.size() ; i++){

		//Still only get the first one

		for(size_t j = 0 ; j < _cluster[i].size() ; j++){
			deque_vertex_all.push_back(_cluster[i][j].getFirst());
		}

	}

	for(size_t j = 0 ; j < hyp_in.size() ; j++){
		deque_vertex_all_model.push_back(hyp_in[j].getSecond());
	}

	for(size_t i = 0 ; i < _cluster.size() ; i++){

		deque_vertex_all_model.push_back(_cluster[i][0].getSecond());
		for(size_t j = 0 ; j < _cluster[i].size() ; j++){
			deque_vertex_all_model.push_back(_cluster[i][j].getSecond());
		}

	}


	gp.labelAll(deque_vertex_all);
	_sf.removeAllFalseLabel(fp, gp);
	gp_model.labelAll(deque_vertex_all_model);
	_sf_model.removeAllFalseLabel(fp_model, gp);

}


/*
 * DO NOT USE
 *
 */
void AASS::graphmatch::Cluster::reduceTEST(graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model)
{


	//TODO : THIS IS ULTRA DUMB BUT I CAN'T SEEM TO COPY TO FILTEREDGRAPH CLASS
	boost::keep_all _keep_all;
	graphmatch::Filtered_place fp(gp.getGraph(), _keep_all, boost::bind(&StorageFilter::removedVertices, &_sf, _1));
	graphmatch::Filtered_place fp_model(gp_model.getGraph(), _keep_all, boost::bind(&StorageFilter::removedVertices, &_sf_model, _1));
// 		std::deque<VertexPlace> deque_vertex;
	//SUPPRESS ALL VERTEX NOT IN HYPO

	std::deque<VertexPlace> deque_vertex_all;
	std::deque<VertexPlace> deque_vertex_all_model;

	//Get only the first one since it's the reduced hypo
	//Get all vertex in the hypo
	for(size_t j = 0 ; j < hyp_in.size() ; j++){
		deque_vertex_all.push_back(hyp_in[j].getFirst());
	}


	//For all hypothesis in cluster reduce the graph
	for(size_t i = 0 ; i < _cluster.size() ; i++){

		//Still only get the first one

		for(size_t j = 0 ; j < _cluster[i].size() ; j++){
			deque_vertex_all.push_back(_cluster[i][j].getFirst());
		}

	}

	for(size_t j = 0 ; j < hyp_in.size() ; j++){
		deque_vertex_all_model.push_back(hyp_in[j].getSecond());
	}

	for(size_t i = 0 ; i < _cluster.size() ; i++){

		deque_vertex_all_model.push_back(_cluster[i][0].getSecond());
		for(size_t j = 0 ; j < _cluster[i].size() ; j++){
			deque_vertex_all_model.push_back(_cluster[i][j].getSecond());
		}

	}

	gp.labelAll(deque_vertex_all);
	_sf.removeAllFalseLabel(fp, gp);
	gp_model.labelAll(deque_vertex_all_model);
	_sf_model.removeAllFalseLabel(fp_model, gp);


	cv::Mat draw = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);
	_sf.drawSpecialFiltered(fp, draw);
	cv::imshow("reduced", draw);

	draw = cv::Mat::zeros(cv::Size(500, 500), CV_8UC3);
	_sf_model.drawSpecialFiltered(fp_model, draw);
	cv::imshow("reduced model", draw);

	cv::waitKey(0);
}


bool AASS::graphmatch::Cluster::isSame(const graphmatch::Hypothese& hyp_in, graphmatch::GraphPlace& gp, graphmatch::Filtered_place& fp, graphmatch::GraphPlace& gp_model, graphmatch::Filtered_place& fp_model) const
{

	bool isSame = true;
	for(size_t i = 0 ; i < hyp_in.size() ; i++){

		std::deque< std::pair< EdgePlace, VertexPlace > > edgevertex;
		std::deque< std::pair< EdgePlace, VertexPlace > > edgevertex_model;

		_sf.getAllEdgeLinked(hyp_in[i].getFirst(), fp, edgevertex);
		_sf_model.getAllEdgeLinked(hyp_in[i].getSecond(), fp_model, edgevertex_model);

		std::deque< std::pair< EdgePlace, VertexPlace > > ev_small;
		std::deque< std::pair< EdgePlace, VertexPlace > > ev_big;

		if(edgevertex.size() < edgevertex_model.size()){
			ev_small = edgevertex;
			ev_big = edgevertex_model;
		}
		else{
			ev_small = edgevertex_model;
			ev_big = edgevertex;
		}


		bool exist = false;

		for(size_t j = 0 ; j < ev_small.size() ; j++){
			for(size_t u = 0 ; u < ev_big.size() ; u++){

				Match m(ev_small[j].second, ev_big[u].second);
				
				if(hyp_in.isSeen(m) == true){
					exist = true;
				}

			}

			if(exist == false){
				isSame = false;
			}

			exist = false;

		}

	}


	return isSame;

}

