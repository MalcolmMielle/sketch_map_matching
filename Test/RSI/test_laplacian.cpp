#include <iostream>
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>

#include "LaplacianGraphMatching/GraphLaplacian.hpp"


BOOST_AUTO_TEST_CASE(trying) {

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

	auto[eigenvalues, eigenvectors] = graph.eigenLaplacian();
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

	auto[eigenvalues2, eigenvectors2] = graph.eigenLaplacian();
	std::cout << "Eigenvalues\n" << eigenvalues2.transpose() << "\neigenvectors\n" << eigenvectors2 << "\n"
	          << std::endl;


	//** Heat kernel **/
	for (int i = 0; i < 10; ++i) {
		graph.propagateHeatKernel(i);

		std::cout << "Heat kernel value of node 0 at time " << i << " : " << graph.getHeatKernelValueNode(vertex, i) << std::endl;
		std::cout << "Heat kernel value of node 0 at time " << i << " : " << graph[vertex].getHeat() << std::endl;
		std::cout << "Heat kernel value of node 1 at time " << i << " : " << graph.getHeatKernelValueNode(vertex2, i) << std::endl;
		std::cout << "Heat kernel value of node 1 at time " << i << " : " << graph[vertex2].getHeat() << std::endl;
		std::cout << "Heat kernel value of node 2 at time " << i << " : " << graph.getHeatKernelValueNode(vertex3, i) << std::endl;
		std::cout << "Heat kernel value of node 2 at time " << i << " : " << graph[vertex3].getHeat() << "\n" << std::endl;
	}







}