#include "LaplacianGraphMatching/GraphLaplacian.hpp"

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
	if(_eigenvalues(0) < 0.)
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