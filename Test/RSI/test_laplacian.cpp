#include <iostream>
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>

#include "LaplacianGraphMatching/GraphLaplacian.hpp"


BOOST_AUTO_TEST_CASE(trying)
{

	AASS::graphmatch::GraphLaplacian graph;

	AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex;
	AASS::graphmatch::Region region;
	region.setUniqueness(0.1);
	graph.addVertex(vertex, region);

	AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex2;
	AASS::graphmatch::Region region2;
	region2.setUniqueness(0.5);
	graph.addVertex(vertex2, vertex, region2);

	std::cout << "Graph created " << std::endl;

	Eigen::MatrixXd ad = graph.getAdjancyMatrix();
	std::cout << "Adjancy matrix\n\n" << ad << "\n" << std::endl;

	Eigen::MatrixXd wl = graph.getWeightedGeneralizedLaplacian();
	std::cout << "Weighted Laplacian\n\n" << wl << "\n" << std::endl;

	auto [eigenvalues, eigenvectors] = graph.eigenLaplacian();
	std::cout << "Eigenvalues\n" << eigenvalues.transpose() << "\neigenvectors\n" << eigenvectors << "\n" << std::endl;



	AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex3;
	AASS::graphmatch::Region region3;
	region3.setUniqueness(1);
	graph.addVertex(vertex3, vertex, region3);

	std::cout << "Graph created " << std::endl;

	Eigen::MatrixXd ad2 = graph.getAdjancyMatrix();
	std::cout << "Adjancy matrix\n\n" << ad2 << "\n" << std::endl;

	Eigen::MatrixXd wl2 = graph.getWeightedGeneralizedLaplacian();
	std::cout << "Weighted Laplacian\n\n" << wl2 << "\n" << std::endl;

	auto [eigenvalues2, eigenvectors2] = graph.eigenLaplacian();
	std::cout << "Eigenvalues\n" << eigenvalues2.transpose() << "\neigenvectors\n" << eigenvectors2 << "\n" << std::endl;


	//Give to each node its eigen vector and eigen value
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
	for (vp = boost::vertices(graph); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex_in = *vp.first;
		std::cout << "Vertex index " << graph[vertex_in].index << " eigen value " << graph[vertex_in].getEigenValue() << " eigenvector" << graph[vertex_in].getEigenVector().transpose() << std::endl;
	}


}