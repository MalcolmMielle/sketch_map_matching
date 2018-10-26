#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"

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
			assert((*this)[vertex].getUniqueness() != -1);
			sum_neighboring_uniqueness = sum_neighboring_uniqueness + (*this)[vertex].getUniqueness();
		}
		generalized_laplacian(index_count, index_count) = sum_neighboring_uniqueness;

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

		double uniqueness_target = target_region.getUniqueness();
		double uniqueness_source = source_region.getUniqueness();
		double weight = - std::sqrt(uniqueness_target * uniqueness_source);

		generalized_laplacian(index_source, index_target) = weight;
		generalized_laplacian(index_target, index_source) = weight;

	}

	//Check that the matrix is symmetric so that we can use the self adjoint solver
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