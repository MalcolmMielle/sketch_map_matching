#ifndef SKETCHALGORITHMS_GRAPHLAPLACIAN_251108
#define SKETCHALGORITHMS_GRAPHLAPLACIAN_251108

#include "bettergraph/SimpleGraph.hpp"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Eigenvalues>

namespace AASS {
	namespace graphmatch {


		class Region {
		protected:
			double _uniqueness = -1;

//			double _eigenvalue;
//			Eigen::VectorXd _eigenvector;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			int index = -1;
			Region(){}

			double getUniqueness() const {return _uniqueness;}
			void setUniqueness(double se){_uniqueness = se;}


			double heatKernel( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time) {
				double score = 0;
				for (int i = 0; i < eigenvalues.size(); ++i){
					double eigenvalue = eigenvalues[i];
					double eigenvectorvalue = eigenvectors.col(i)(index);
					score = score + std::exp(-time * eigenvalue) * (eigenvectorvalue * eigenvectorvalue);
				}
				return score;
			}

//			void setEigen(double value, const Eigen::VectorXd& vector){
//				_eigenvalue = value;
//				_eigenvector = vector;
//			}

//			double getEigenValue(){return _eigenvalue;}
//			const Eigen::VectorXd& getEigenVector() const {return _eigenvector;}

		};

		class EdgeType {
		public:
			EdgeType(){};
		};


		class GraphLaplacian : public bettergraph::SimpleGraph<Region, EdgeType> {

		protected:

			Eigen::VectorXd _eigenvalues;
			Eigen::MatrixXd _eigenvectors;

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::GraphType GraphLaplacianType;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::Vertex VertexLaplacian;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::Edge EdgeLaplacian;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::VertexIterator VertexIteratorLaplacian;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::EdgeIterator EdgeIteratorLaplacian;


			GraphLaplacian() {}

			/**
			 *
			 * @return the adjancy matrix with the sources as the rows and the target as columns
			 */
			Eigen::MatrixXd getAdjancyMatrix();
			Eigen::MatrixXd getWeightedGeneralizedLaplacian();

			std::tuple<Eigen::VectorXd, Eigen::MatrixXd> eigenLaplacian();

			void laplacianFamilySignatureGeneration(){};

			double getHeatKernelValueNode(const VertexLaplacian& vertex, double time){
				return (*this)[vertex].heatKernel(_eigenvalues, _eigenvectors, time);
			}

		};

	}
}

#endif