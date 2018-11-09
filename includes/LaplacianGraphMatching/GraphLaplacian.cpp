#include "GraphLaplacian.hpp"
#include "MatchLaplacian.hpp"
#include "HypotheseLaplacian.hpp"


int AASS::graphmatch::GraphLaplacian::editDistance(const GraphLaplacian::VertexLaplacian& v, const std::deque< Region>& other_graph_neighbor, const std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > >& all_edge_other_graph, std::deque< AASS::graphmatch::MatchLaplacian >& out, std::string& operation_out) const
{

	std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > > all_edge;
	int best_dist = -1;

	getAllEdgeLinkedCounterClockWise(v, all_edge);
	std::deque< Region> all_places;

	for(auto it = all_edge.begin() ; it != all_edge.end() ; ++it){
		all_places.push_back( (*this)[it->second] );
	}

// 			getAllVertexAttrCounterClockWise(v, all_places);

	//Make a string out of the order of vertex.
// 			std::string string = makeString(all_edge);
	std::string operation;
	std::string original;
	int starting_point = -1;

	//compare every possibility.
	for(size_t i = 0 ; i < all_places.size() ; i++){
		size_t y = i;
// 				std::deque<VertexPlace> list;


		std::deque< Region> new_test;

// 				std::string new_test;
		//Get all string combination
		for(size_t j = 0 ; j < all_places.size() ; j++){
			new_test.push_back(all_places[y]);
			y++;
// 					list.push_back(all_edge[y].second);
			if(y == all_places.size()){
				y = 0;
			}
		}

		std::string out_string;

		std::function<bool(Region, Region)> compareFunction = graphmatch::compareRegion;


// 				(const std::deque<ModifyTypeElement>& string_to_modify, const std::deque<UnchangedTypeElement>& unchanged, std::function<bool(ModifyTypeElement, UnchangedTypeElement)> compareFunction, std::string& out)

		double edistance = AASS::editdistance::normalizedEditDistance<Region, Region>(new_test, other_graph_neighbor, compareFunction, out_string);

// 				std::cout << "the out string " << out_string << " for " << new_test <<" and " <<other(*this)_neighbor <<std::endl;
		// 			std::cout << "case : " << new_test;
		// 			std::cout << " distance : " << edistance << std::endl;
		if(best_dist == -1 || best_dist > edistance){
			best_dist = edistance;
			operation = out_string;
			//TODO Useless
// 					original = new_test;
			starting_point = y;

		}

	}

	size_t place_index = starting_point;
	int model_index = 0;

	/*
	* Interesting part is here
	*
	*
	*/

	for(size_t i = 0 ; i < operation.size(); i++){

		//Match
		if(operation.at(i) == 'n'){

			out.push_back(
					AASS::graphmatch::MatchLaplacian(
							all_edge[place_index].second,
							all_edge_other_graph[model_index].second)
			);

			place_index++;
			model_index++;
		}
			//Substitution
		else if(operation.at(i) == 's'){

			out.push_back(
					AASS::graphmatch::MatchLaplacian(
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


	}

	// 		std::deque< AASS::graphmatch::MatchLaplacian >::iterator it_test_editdistance;

	//Keep the best one edit distance one
	return best_dist;
}



double AASS::graphmatch::GraphLaplacian::makeMatching(const AASS::graphmatch::GraphLaplacian::VertexLaplacian& v, const AASS::graphmatch::GraphLaplacian::VertexLaplacian& v_model, const AASS::graphmatch::GraphLaplacian& gp_model, const std::deque< AASS::graphmatch::MatchLaplacian >& matched_previously, std::deque< AASS::graphmatch::MatchLaplacian >& out_match){

	// 		std::cout << std::endl << std::endl << " MAKE MATCHING " << std::endl << std::endl;
	//List of the final string combined.
	//Get clockwise edges
	std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > > all_edge;
	this->getAllEdgeLinkedCounterClockWise(v, all_edge);

	// 		std::cout << "Result for real " << std::endl;
	// 		for(size_t i = 0; i < all_edge.size() ; i++){
	// 			gp.print(all_edge[i].second);
	// 		}

	std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > > all_edge_model;
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
			std::deque<AASS::graphmatch::GraphLaplacian::VertexLaplacian> first_neigh;
			std::deque<Region> f_neigh_place;
			this->getNeighborBetween2(pair_matched[i].first, pair_matched[i+1].first, all_edge, first_neigh, f_neigh_place);

// 					getString(pair_matched[i].second, pair_matched[i+1].second, all_edge_model, gp_model, all_strings_model);
			std::deque<AASS::graphmatch::GraphLaplacian::VertexLaplacian> first_neigh_model;
			std::deque<Region> f_neigh_place_model;
			gp_model.getNeighborBetween2(pair_matched[i].second, pair_matched[i+1].second, all_edge_model, first_neigh_model,f_neigh_place_model);

			// 				std::cout << "String result size " << all_strings_first.size() << std::endl;

			//Match every string correspondance
			std::string out;

			std::function<bool(Region, Region)> compareFunction = graphmatch::compareRegion;
			edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance<Region, Region>(f_neigh_place, f_neigh_place_model, compareFunction, out);

// 					edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance(all_strings_first, all_strings_model, out);

			//Create Match
			createMatch(out, all_edge, all_edge_model, pair_matched[i], out_match);
		}
		//Last loop
// 				std::string all_strings_first;
// 				std::string all_strings_model;
// 				getString(pair_matched[pair_matched.size()-1].first, pair_matched[0].first, all_edge, gp, all_strings_first);
// 				getString(pair_matched[pair_matched.size()-1].second, pair_matched[0].second, all_edge_model, gp_model, all_strings_model);

		std::deque<AASS::graphmatch::GraphLaplacian::VertexLaplacian> first_neigh;
		std::deque<Region> f_neigh_place;
		this->getNeighborBetween2(pair_matched[pair_matched.size()-1].first, pair_matched[0].first, all_edge, first_neigh, f_neigh_place);

		std::deque<AASS::graphmatch::GraphLaplacian::VertexLaplacian> first_neigh_model;
		std::deque<Region> f_neigh_place_model;
		gp_model.getNeighborBetween2(pair_matched[pair_matched.size()-1].second, pair_matched[0].second, all_edge_model, first_neigh_model,f_neigh_place_model);

// 				std::cout << "String result size " << all_strings_first.size() << std::endl;

		//Match every string correspondance
		std::string out;

		std::function<bool(Region, Region)> compareFunction = graphmatch::compareRegion;
		edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance<Region, Region>(f_neigh_place, f_neigh_place_model, compareFunction, out);

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

		std::deque<AASS::graphmatch::GraphLaplacian::VertexLaplacian> first_neigh;
		std::deque<Region> f_neigh_place;
		this->getNeighborBetween2(pair_matched[0].first, pair_matched[0].first, all_edge, first_neigh, f_neigh_place);

		std::deque<AASS::graphmatch::GraphLaplacian::VertexLaplacian> first_neigh_model;
		std::deque<Region> f_neigh_place_model;
		gp_model.getNeighborBetween2(pair_matched[0].second, pair_matched[0].second, all_edge_model, first_neigh_model,f_neigh_place_model);

// 				std::cout << "String result size " << all_strings_first.size() << std::endl;

		//Match every string correspondance
		std::string out;

		std::function<bool(Region, Region)> compareFunction = graphmatch::compareRegion;
		edit_distance = edit_distance + AASS::editdistance::normalizedEditDistance<Region, Region>(f_neigh_place, f_neigh_place_model, compareFunction, out);


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


void AASS::graphmatch::GraphLaplacian::getNeighborBetween2(size_t start, size_t end, const std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > >& all_edge, std::deque< AASS::graphmatch::GraphLaplacian::VertexLaplacian >& neighbor, std::deque< AASS::graphmatch::Region >& places) const
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
		places.push_back((*this)[all_edge[start].second]);
		//Move forward
		start++;
		if(start == all_edge.size()){
			// 				std::cout << "INIT TO 0 " << std::endl;
			start = 0 ;
		}
	}

}

void AASS::graphmatch::GraphLaplacian::createMatch(const std::string& operation, const std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > >& all_edge, const std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > >& all_edge_other_graph, std::pair< int, int > start, std::deque< graphmatch::MatchLaplacian >& out){

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
					graphmatch::MatchLaplacian(
							all_edge[place_index].second,
							all_edge_other_graph[model_index].second)
			);

			place_index++;
			model_index++;
		}
			//Substitution
		else if(operation.at(i) == 's'){

			out.push_back(
					graphmatch::MatchLaplacian(
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



Eigen::MatrixXd AASS::graphmatch::GraphLaplacian::getAdjancyMatrix(){

	int number_of_vertices = getNumVertices();
	Eigen::MatrixXd adjancy_matrix = Eigen::MatrixXd::Zero(number_of_vertices, number_of_vertices);

	std::cout << "Init adjancy matrix:\n" << adjancy_matrix << std::endl;

	std::pair<VertexIteratorLaplacian, VertexIteratorLaplacian> vp;
	int index_count = 0;
	for (vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
//		Region region = (*this)[*vp.first];
		if((*this)[*vp.first].index == -1){
			(*this)[*vp.first].index = index_count;
//			assert((*this)[*vp.first].index = index_count);
		}
		index_count++;
	}

	std::cout << "All index added:\n" << adjancy_matrix << std::endl;

	//Use the edges because it's an undirected graph
	auto es = boost::edges(*this);
	for (auto eit = es.first; eit != es.second; ++eit) {
//		std::cout << boost::source(*eit, *this) << ' ' << boost::target(*eit, *this) << std::endl;
		VertexLaplacian source = boost::source(*eit, *this);
		VertexLaplacian target = boost::target(*eit, *this);
		int index_source = (*this)[source].index;
		int index_target = (*this)[target].index;

		assert(index_source != -1);
		assert(index_target != -1);

		std::cout << "Adding to " << index_source << ", " << index_target << std::endl;

		adjancy_matrix(index_source, index_target) = 1;
		adjancy_matrix(index_target, index_source) = 1;

	}

	return adjancy_matrix;

}

Eigen::MatrixXd AASS::graphmatch::GraphLaplacian::getWeightedGeneralizedLaplacian(){

	int number_of_vertices = getNumVertices();
	Eigen::MatrixXd generalized_laplacian = Eigen::MatrixXd::Zero(number_of_vertices, number_of_vertices);
	std::cout << "G\n" << generalized_laplacian << std::endl;

	std::pair<VertexIteratorLaplacian, VertexIteratorLaplacian> vp;
	int index_count = 0;
	for (vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
		VertexLaplacian vertex_in = *vp.first;
//		Region region = (*this)[vertex_in];

		if((*this)[vertex_in].index == -1){
			(*this)[vertex_in].index = index_count;
		}

		std::deque< VertexLaplacian > all_vertices;
		getAllVertexLinked(vertex_in, all_vertices);
		double sum_neighboring_uniqueness = 0;
		for(auto vertex : all_vertices){
			assert((*this)[vertex].getValue() != -1);
//			std::cout << "Get value " << (*this)[vertex].getValue() << std::endl;
			sum_neighboring_uniqueness = sum_neighboring_uniqueness + (*this)[vertex].getValue();
		}
//		std::cout << "Sum " << sum_neighboring_uniqueness << std::endl;
		generalized_laplacian(index_count, index_count) = sum_neighboring_uniqueness;

//		std::cout << "G\n" << generalized_laplacian << std::endl;
		index_count++;
	}

	//Use the edges because it's an undirected graph
	auto es = boost::edges(*this);
	for (auto eit = es.first; eit != es.second; ++eit) {
//		std::cout << boost::source(*eit, *this) << ' ' << boost::target(*eit, *this) << std::endl;
		VertexLaplacian source = boost::source(*eit, *this);
		VertexLaplacian target = boost::target(*eit, *this);
		Region source_region = (*this)[source];
		Region target_region = (*this)[target];

		int index_source = (*this)[source].index;
		int index_target = (*this)[target].index;

		std::cout << "Uniqueness " << target_region.getValue() << " " << source_region.getValue() << std::endl;

		double uniqueness_target = target_region.getValue();
		double uniqueness_source = source_region.getValue();
		assert(uniqueness_target >= 0 );
		assert(uniqueness_source >= 0 );
		double weight = - std::sqrt(uniqueness_target * uniqueness_source);

		generalized_laplacian(index_source, index_target) = weight;

//		std::cout << "G\n" << generalized_laplacian << std::endl;
		generalized_laplacian(index_target, index_source) = weight;
//		std::cout << "G\n" << generalized_laplacian << std::endl;

	}

	//Check that the matrix is symmetric so that we can use the self adjoint solver
	if(generalized_laplacian != generalized_laplacian.transpose()){
		std::cout << "Not symmetric : !\n" << generalized_laplacian << " \n==\n" << generalized_laplacian.transpose() << std::endl;
	}
	assert(generalized_laplacian == generalized_laplacian.transpose());
	return generalized_laplacian;

}



std::tuple<Eigen::VectorXd, Eigen::MatrixXd> AASS::graphmatch::GraphLaplacian::eigenLaplacian() {

	Eigen::MatrixXd laplacian = getWeightedGeneralizedLaplacian();

	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver;
	eigen_solver.compute(laplacian);

	_eigenvalues = eigen_solver.eigenvalues();
	_eigenvectors = eigen_solver.eigenvectors();

	//CHeck Semi Positive Definite. First eigen value is the smallest
	if(std::round(_eigenvalues(0) * 1000) < 0.)
	{
		std::cout << "Laplacian\n" << laplacian << "\n\neigen values\n" << _eigenvalues.transpose() << std::endl;
		throw std::runtime_error("non semi-positive definite matrix!");
	}

	//Give to each node its eigen vector and eigen value - NOT AT ALL HOW THIS WORKS
//	std::pair<VertexIteratorLaplacian, VertexIteratorLaplacian> vp;
//	for (vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
//		VertexLaplacian vertex_in = *vp.first;
//		assert((*this)[vertex_in].index != -1);
//		int index = (*this)[vertex_in].index;
//		(*this)[vertex_in].setEigen(eigenvalues(index), eigenvectors.col(index));
//	}

	return std::make_tuple(_eigenvalues, _eigenvectors);

}


std::vector<AASS::graphmatch::MatchLaplacian> AASS::graphmatch::GraphLaplacian::compare(const AASS::graphmatch::GraphLaplacian& gl) const {

	std::cout << "Comparing two graph Laplacian" << std::endl;

	std::vector<AASS::graphmatch::MatchLaplacian> out;
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
	//vertices access all the vertix
	for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
		auto v = *vp.first;
// 		std::cout << "new source" << std::endl;
		std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp_target;
		//vertices access all the vertix
		for (vp_target = boost::vertices(gl); vp_target.first != vp_target.second; ++vp_target.first) {
			auto v_target = *vp_target.first;

			double score = (*this)[v].compare(gl[v_target]);
//			std::cout << "SCORE " << score << std::endl;
			AASS::graphmatch::MatchLaplacian match(v, v_target);
			match.setCost(score);
			out.push_back(match);
		}
	}
	return out;
}


std::vector< AASS::graphmatch::MatchLaplacian > AASS::graphmatch::GraphLaplacian::hungarian_matching(const AASS::graphmatch::GraphLaplacian& laplacian_model){

	auto res = this->compare(laplacian_model);

	std::cout << "Wat" << std::endl;
	for(size_t i = 0 ; i < res.size() ; ++i){
		std::cout << res[i].getFirst() << " " << res[i].getSecond() << std::endl;
	}

	std::vector<int> simi;
	auto it = res.begin();
	for ( ; it != res.end() ; ++it){
		int input = it->getCost()*100;
		std::cout << " pushing " << input << " because " << it->getCost()*100 << std::endl;
		assert(input <= 100);
		assert(input >= 0);
		simi.push_back(input);
	}

	assert((int)simi.size() == this->getNumVertices() * laplacian_model.getNumVertices());


	/** Lambda **/
	auto array_to_matrix = [] (const std::vector<int>& m, int rows, int cols) -> int** {
		int i,j;
		int** r;
		r = (int**)calloc(rows,sizeof(int*));
		for(i=0;i<rows;i++)
		{
			r[i] = (int*)calloc(cols,sizeof(int));
			for(j=0;j<cols;j++)
				r[i][j] = m[i*cols+j];
		}
		return r;
	};

//	int** array_to_matrix(const std::vector<int>& m, int rows, int cols)




	std::cout << "OUT source on rows and target on cols" << std::endl;
	int** m = array_to_matrix(simi, this->getNumVertices(), laplacian_model.getNumVertices());

	hungarian_problem_t p;
//	int matrix_size = hungarian_init(&p, m , this->getNumVertices(), laplacian_model.getNumVertices(), HUNGARIAN_MODE_MINIMIZE_COST);
	hungarian_init(&p, m , this->getNumVertices(), laplacian_model.getNumVertices(), HUNGARIAN_MODE_MINIMIZE_COST);

	/* some output */
	fprintf(stderr, "cost-matrix:");
	hungarian_print_costmatrix(&p);

	std::cout << "Solving" << std::endl;
	/* solve the assignement problem */
	hungarian_solve(&p);
	std::cout << "Solving done" << std::endl;

	/* some output */
	fprintf(stderr, "assignment:");
	hungarian_print_assignment(&p);

	/* some output */
	fprintf(stderr, "cost-matrix:");
	hungarian_print_costmatrix(&p);

	std::vector< MatchLaplacian > hungarian_matches;
	std::vector<int> scores;
	// This depend on which one as more nodes !
	// Goes along the line
	if(this->getNumVertices() <= laplacian_model.getNumVertices()){

		int i,j;
		// 			fprintf(stderr , "\n");
		for(i=0; i<this->getNumVertices(); i++) {
			// 				fprintf(stderr, " [");
			std::cout << " i " << std::endl;
			for(j=0; j<laplacian_model.getNumVertices(); j++) {

				std::cout << " w " << std::endl;
				if(p.assignment[i][j] == 1){

					std::cout << "Matching lol2" << i << " with " << j << " cost " << simi.at( ( i*laplacian_model.getNumVertices() ) + j) << std::endl;

					std::cout << "Matching " << i * laplacian_model.getNumVertices() << " with " << (i * laplacian_model.getNumVertices() )  + j << std::endl;
					std::cout << res.at(i * laplacian_model.getNumVertices()).getFirst() << " " << res.at(( i * laplacian_model.getNumVertices() ) + j).getSecond() << std::endl;

// 							ZoneCompared m(res.at(i * laplacian_model.getNumVertices()).source, res.at(( i * laplacian_model.getNumVertices() ) + j).target, simi.at( ( i*laplacian_model.getNumVertices() ) + j));

					assert(res.at(( i * laplacian_model.getNumVertices() ) + j).getFirst() == res.at(i * laplacian_model.getNumVertices()).getFirst());
					std::cout << "u" << std::endl;
					assert(simi.at( ( i*laplacian_model.getNumVertices() ) + j) == (int) (res.at(( i * laplacian_model.getNumVertices() ) + j).getCost() * 100 ));
					std::cout << "u" << std::endl;

					hungarian_matches.push_back(res.at(( i * laplacian_model.getNumVertices() ) + j));
					std::cout << "u" << std::endl;
					scores.push_back(simi.at( ( i*laplacian_model.getNumVertices() ) + j));
					std::cout << "u" << std::endl;
				}
			}


			// 				fprintf(stderr, "]\n");
		}
	}
		//Goes down the column
	else{
		assert(hungarian_matches.size() == 0);
		std::cout << "Source more than target" << std::endl;
		int i,j;
		// 			fprintf(stderr , "\n");
		std::cout << "target " << laplacian_model.getNumVertices() << " source " << this->getNumVertices() << std::endl;
		for(i = 0; i < laplacian_model.getNumVertices(); i++) {
			// 				fprintf(stderr, " [");
			for(j = 0; j < this->getNumVertices(); j++) {
// 						std::cout << " ass " << p.assignment[j][i] << std::endl;
				if(p.assignment[j][i] == 1){

					std::cout << "Matching lol " << j << " with " << i << " cost " << simi.at( j * laplacian_model.getNumVertices()) + i << std::endl;

					std::cout << "Matching " <<j * laplacian_model.getNumVertices()  +i << std::flush << " with " <<  j * laplacian_model.getNumVertices() + i << std::endl;

					std::cout << res.at(j * laplacian_model.getNumVertices() + i).getFirst() << " " << res.at(j * laplacian_model.getNumVertices() + i).getSecond() << std::endl;

// 							ZoneCompared m(res.at((j * laplacian_model.getNumVertices()) + i).source,
// 										   res.at((j * laplacian_model.getNumVertices()) + i).target,
// 										   simi.at( ( i*source.getNumUnique() ) + j));
// 							hungarian_matches.push_back(m);
// 							hungarian_matches.push_back(std::pair<GraphZoneRI::Vertex, GraphZoneRI::Vertex>(
// 								res.at((j * laplacian_model.getNumVertices()) + i).source,
// 								res.at((j * laplacian_model.getNumVertices()) + i).target)
// 							);

// 							std::cout << simi.at( ( i*source.getNumUnique() ) + j) <<"==" << (int) (res.at((j * laplacian_model.getNumVertices()) + i).getSimilarity() * 100 )<< std::endl;

// 							assert(simi.at( ( i*source.getNumUnique() ) + j) == (int) (res.at((j * laplacian_model.getNumVertices()) + i).getSimilarity() * 100 ) );

					hungarian_matches.push_back( res.at((j * laplacian_model.getNumVertices()) + i) );

					scores.push_back(simi.at( ( i * this->getNumVertices() ) + j));
				}
			}


			// 				fprintf(stderr, "]\n");
		}
	}
// 			fprintf(stderr, "\n");
	std::cout << " hungarian_matches " << std::endl;
//Freeing the memory
	int idx;
	for (idx = 0; idx < this->getNumVertices(); idx += 1) {
		std::cout << "free" << std::endl;
		free(m[idx]);
		std::cout << " afterfree" << std::endl;
	}
	std::cout << "final free" << std::endl;
	free(m);


	std::cout << "outout" << std::endl;
	std::cout << "return " <<hungarian_matches.size() << std::endl;

	for(size_t i = 0 ; i < hungarian_matches.size() ; ++i){
		std::cout << "matching " << i << " : " << hungarian_matches[i].getFirst() << " " << hungarian_matches[i].getSecond() << std::endl;
	}

	std::sort(hungarian_matches.begin(), hungarian_matches.end(), [](AASS::graphmatch::MatchLaplacian &match, AASS::graphmatch::MatchLaplacian &match1){
//		return match.getRanking(graph_slam, graph_slam2) > match1.getRanking(graph_slam, graph_slam2);
		return match.getCost() < match1.getCost();
	} );

	return hungarian_matches;


}

void AASS::graphmatch::GraphLaplacian::pairWiseMatch(
		const graphmatch::GraphLaplacian& gp2,
		std::deque<graphmatch::MatchLaplacian >& places_pair)
{


	std::pair<GraphLaplacian::VertexIteratorLaplacian, GraphLaplacian::VertexIteratorLaplacian> vp;
	for (vp = boost::vertices(getGraph()); vp.first != vp.second; vp.first++) {

		GraphLaplacian::VertexLaplacian v = *vp.first;
// 		bool is_room = gp.isRoom(v);
//					std::string is_type = (*this)[v].getID();

		std::pair<GraphLaplacian::VertexIteratorLaplacian, GraphLaplacian::VertexIteratorLaplacian> vp2;
		for (vp2 = boost::vertices(gp2.getGraph()); vp2.first != vp2.second; vp2.first++) {

			GraphLaplacian::VertexLaplacian v2 = *vp2.first;

// 			std::deque< std::pair < topologicalmap::EdgePlace, VertexPlace > > all_linked_edge;
// 			gp.getAllEdgeLinked(v, all_linked_edge);
//
// 			std::deque< std::pair < topologicalmap::EdgePlace, VertexPlace > > all_linked_edge2;
// 			gp.getAllEdgeLinked(v2, all_linked_edge2);

// 			bool is_room_2 = gp2.isRoom(v2);
//						std::string is_type_model = gp2[v2].getID();

			//ATTENTION
			//Same number of linked room + same type of room. Maybe it's too big an assumption that they have the same number of links
// 			if(/*all_linked_edge.size() == all_linked_edge2.size() &&*/ is_room_2 == is_room){
			if( (*this)[v].compareBool(gp2[v2]) ){
				//Adding to the list of the vertex are of the same type
				places_pair.push_back(graphmatch::MatchLaplacian(v, v2) );
			}

		}

	}
}