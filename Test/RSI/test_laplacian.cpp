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
	region.setValue(0.1);
	graph.addVertex(vertex, region);

	AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex2;
	AASS::graphmatch::Region region2;
	region2.setValue(0.5);
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
	region3.setValue(1);
	graph.addVertex(vertex3, vertex, region3);

	std::cout << "Graph created " << std::endl;

	Eigen::MatrixXd ad2 = graph.getAdjancyMatrix();
	std::cout << "Adjancy matrix\n\n" << ad2 << "\n" << std::endl;

	Eigen::MatrixXd wl2 = graph.getWeightedGeneralizedLaplacian();
	std::cout << "Weighted Laplacian\n\n" << wl2 << "\n" << std::endl;

	auto[eigenvalues2, eigenvectors2] = graph.eigenLaplacian();
	std::cout << "Eigenvalues\n" << eigenvalues2.transpose() << "\neigenvectors\n" << eigenvectors2 << "\n"
	          << std::endl;

	graph.addAnchor(vertex2);

	//** Heat kernel **/
	for (int i = 0; i < 10; ++i) {
		graph.propagateHeatKernel(i);

		std::cout << "Heat kernel value of node 0 at time " << i << " : " << graph.getHeatKernelValueNode(vertex, i) << std::endl;
		std::cout << "Heat kernel value of node 0 at time " << i << " : " << graph[vertex].getHeat() << std::endl;
		std::cout << "Anchor heat kernel value of node 0 at time " << i << " : " << graph[vertex].getHeatAnchors() << std::endl;
		std::cout << "Heat kernel value of node 1 at time " << i << " : " << graph.getHeatKernelValueNode(vertex2, i) << std::endl;
		std::cout << "Heat kernel value of node 1 at time " << i << " : " << graph[vertex2].getHeat() << std::endl;
		std::cout << "Anchor heat kernel value of node 1 at time " << i << " : " << graph[vertex2].getHeatAnchors() << std::endl;
		std::cout << "Heat kernel value of node 2 at time " << i << " : " << graph.getHeatKernelValueNode(vertex3, i) << std::endl;
		std::cout << "Heat kernel value of node 2 at time " << i << " : " << graph[vertex3].getHeat() << std::endl;
		std::cout << "Anchor heat kernel value of node 2 at time " << i << " : " << graph[vertex3].getHeatAnchors() << "\n" << std::endl;
	}



	AASS::graphmatch::GraphLaplacian graph2;

	AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex_g2;
	AASS::graphmatch::Region region_g2;
	region_g2.setValue(0.1);
	graph2.addVertex(vertex_g2, region_g2);

	AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex2_g2;
	AASS::graphmatch::Region region2_g2;
	region2_g2.setValue(0.5);
	graph2.addVertex(vertex2_g2, vertex_g2, region2_g2);



	std::cout << "SImilarity:\n" << graph.getSimilarity(graph2) << std::endl;
	std::cout << "Chi square:\n" << graph.getChiSquare(graph2) << std::endl;
	std::cout << "Bhattacha:\n" << graph.getBhattacharyyaDistance(graph2) << std::endl;
	std::cout << "mAHALANOBIS:\n" << graph.getMahalanobisDistance(graph2) << std::endl;

	BOOST_ASSERT(graph.getMahalanobisDistance(graph2) == 1);

	std::cout << "mAHALANOBIS likelihood:\n" << graph.getMahalanobisLikelihood(graph2) << std::endl;

	std::cout << "DONE" << std::endl;

}