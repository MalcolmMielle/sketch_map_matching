#ifndef SKETCHALGORITHMS_GRAPHLAPLACIAN_251108
#define SKETCHALGORITHMS_GRAPHLAPLACIAN_251108

#include "RSI/ZoneRI.hpp"
#include "RSI/hungarian/hungarian.h"
#include "bettergraph/SimpleGraph.hpp"
#include "eigen3/Eigen/Core"
#include <eigen3/Eigen/Eigenvalues>
#include <opencv2/opencv.hpp>

#include <editDistance/NormalizedEditDistance.hpp>

namespace AASS {
	namespace graphmatch {


		class MatchLaplacian;
		class HypotheseLaplacian;


		class Region {
		protected:

			std::vector< std::vector< cv::Point > > _contour;
//			std::deque <cv::Point2i> _zone;
//			cv::Moments moment;
			cv::Point2f _center;

			bool _use_heat_anchors = true;

			double _value_vertex = -1;

			double _heat = -1;
			double _time = -1;

			double _heat_anchors = -1;

//			double _eigenvalue;
//			Eigen::VectorXd _eigenvector;

		public:
			bool label = false;
			double _threshold_same = 0.05;

			AASS::RSI::ZoneRI zone;


			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			int index = -1;
			Region(){}

			Region(const Region& r){
				_contour = r.getContour();
				_center = r.getCenter();
				_value_vertex = r.getValue();
				_heat = r.getHeat();
				_time = r.getTime();
				_heat_anchors = r.getHeatAnchors();
				zone = r.zone;
			}

			void useHeatAnchors(bool b){_use_heat_anchors = b;}
			bool useHeatAnchors() const {return _use_heat_anchors;}

			void print() const {
				std::cout << "Node " << index << " heat " << _heat << " heat anchors " << _heat_anchors << " time " << _time << std::endl;
			}

			void setCenter(const cv::Point2f& center){_center = center;}
			void setContour(const std::vector< std::vector< cv::Point > >& contour){_contour = contour;}

			const cv::Point2f& getCenter() const {return _center;}
			const std::vector< std::vector< cv::Point > >& getContour() const {return _contour;}

			double getValue() const {return _value_vertex;}
			void setValue(double se){_value_vertex = se;}

			double getHeat() const {return _heat;}
			double getHeatAnchors() const {return _heat_anchors;}
			double getTime() const {return _time;}

			double heatKernel( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time) {
				double score = 0;
				for (int i = 0; i < eigenvalues.size(); ++i){
					double eigenvalue = eigenvalues[i];
					double eigenvectorvalue = eigenvectors.col(i)(index);
					score = score + std::exp(-time * eigenvalue) * (eigenvectorvalue * eigenvectorvalue);
				}

				_heat = score;
				_time = time;
				return score;
			}

			double heatKernelAnchor( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time, double index_anchor) {
				double score = 0;
				for (int i = 0; i < eigenvalues.size(); ++i){
					double eigenvalue = eigenvalues[i];
					double eigenvectorvalue = eigenvectors.col(i)(index);
					double eigenvectorvalue_anchor = eigenvectors.col(i)(index_anchor);
					score = score + std::exp(-time * eigenvalue) * (eigenvectorvalue * eigenvectorvalue_anchor);
				}
				return score;
			}

			double heatKernelAnchors( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time, std::deque<int> indexes_anchor) {
				double score_anchors = 0;
				for(auto index : indexes_anchor) {
					score_anchors = score_anchors + heatKernelAnchor(eigenvalues, eigenvectors, time, index);
				}
				_heat_anchors = score_anchors;
				heatKernel(eigenvalues, eigenvectors, time);
				return score_anchors;
			}

			double compare(const Region& region) const {
				assert(region.getTime() == _time);
				assert(_heat_anchors != -1);
//				std::cout << "Heats : " << region.getHeatAnchors() << " - " << _heat_anchors << std::endl;
//				return std::abs( region.getHeatAnchors() - _heat_anchors );
				if (_use_heat_anchors) {
					return std::abs(region.getHeatAnchors() - _heat_anchors);
				} else {
					return std::abs(region.getHeat() - _heat);
				}
			}

//			void setEigen(double value, const Eigen::VectorXd& vector){
//				_eigenvalue = value;
//				_eigenvector = vector;
//			}

//			double getEigenValue(){return _eigenvalue;}
//			const Eigen::VectorXd& getEigenVector() const {return _eigenvector;}


			bool compareBool(const Region& r) const {
				assert(r.getTime() == _time);
				assert(_heat_anchors != -1);

				if(_use_heat_anchors){
//					std::cout << "HEAT " << r.getHeatAnchors() << " " << _heat_anchors << std::endl;
					if(r.getHeatAnchors() <= _heat_anchors + _threshold_same && r.getHeatAnchors() >= _heat_anchors - _threshold_same){
//						std::cout << "True" << std::endl;
						return true;
					}
				}
				else{
					if(r.getHeat() <= _heat + _threshold_same && r.getHeat() >= _heat - _threshold_same){
//						std::cout << "True" << std::endl;
						return true;
					}
				}
//				std::cout << "False" << std::endl;
				return false;

			}



		};


		inline std::ostream& operator<<(std::ostream& in, const Region &p){

			in << p.getCenter();
			return in;

		}

		inline bool operator==(const Region& in, const Region &p){

			return in.getHeatAnchors() == p.getHeatAnchors();

		}

		inline bool compareRegion(const Region& p, const Region& p2){
			return p.compareBool(p2);
		}







		class EdgeType {
		public:
			EdgeType(){};
		};


		class GraphLaplacian : public bettergraph::SimpleGraph<Region, EdgeType> {

		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::GraphType GraphLaplacianType;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::Vertex VertexLaplacian;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::Edge EdgeLaplacian;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::VertexIterator VertexIteratorLaplacian;
			typedef typename bettergraph::SimpleGraph<Region, EdgeType>::EdgeIterator EdgeIteratorLaplacian;

		protected:

			Eigen::VectorXd _eigenvalues;
			Eigen::MatrixXd _eigenvectors;
			std::deque<VertexLaplacian> _anchors;

		public:


			GraphLaplacian() {}
			GraphLaplacian( const GraphLaplacian& ) = delete; // non construction-copyable
			GraphLaplacian& operator=( const GraphLaplacian& ) = delete; // non copyable

			void print() const {

				for(auto anchor : _anchors){
					std::cout << "Anchor index " << (*this)[anchor].index << " ";
				}
				std::cout << std::endl;


				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
				//vertices access all the vertix
				for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
					auto v = *vp.first;
					(*this)[v].print();
				}
			}

			void useHeatAnchors(bool b){
				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
				//vertices access all the vertix
				for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
					auto v = *vp.first;
					(*this)[v].useHeatAnchors(b);
				}
			}

			void addAnchor(const VertexLaplacian& anch){_anchors.push_back(anch);}


			void noWeightForVertices(){
				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp3;
				for (vp3 = boost::vertices(*this); vp3.first != vp3.second; ++vp3.first) {
					auto v = *vp3.first;
					(*this)[v].setValue(1);
				}
			}

			void useUniquenessScoreAsWeights(){
				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp3;
				for (vp3 = boost::vertices(*this); vp3.first != vp3.second; ++vp3.first) {
					auto v = *vp3.first;
					double uniqueness = (*this)[v].zone.getUniquenessScore();
					(*this)[v].setValue(uniqueness);
				}
			}

			void useRelativeSizeAsWeights(){
//				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp3;
				auto vp3 = boost::vertices(*this);
				auto v = *vp3.first;
				double lowest_value = (*this)[v].zone.getSizeClassification();

				for (; vp3.first != vp3.second; ++vp3.first) {
					v = *vp3.first;
					double size_class = (*this)[v].zone.getSizeClassification();
					if(size_class < lowest_value){
						lowest_value = size_class;
					}
				}
				for (vp3 = boost::vertices(*this); vp3.first != vp3.second; ++vp3.first) {
					v = *vp3.first;
					double size_class = (*this)[v].zone.getSizeClassification();
					(*this)[v].setValue(size_class - lowest_value);
				}
			}

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

			void propagateHeatKernel(double time){

				std::deque<int> index_anchors;
				for(auto anchor : _anchors){
					index_anchors.push_back( (*this)[anchor].index );
				}

				std::pair<VertexIteratorLaplacian, VertexIteratorLaplacian> vp;
				for (vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
					VertexLaplacian vertex_in = *vp.first;
					(*this)[vertex_in].heatKernelAnchors(_eigenvalues, _eigenvectors, time, index_anchors);
				}
			}

			std::vector<AASS::graphmatch::MatchLaplacian> compare(const GraphLaplacian& gl) const ;

			std::vector< AASS::graphmatch::MatchLaplacian > hungarian_matching(const AASS::graphmatch::GraphLaplacian& laplacian_model);


			void drawSpecial(cv::Mat& m, const VertexLaplacian& v, const cv::Scalar& color ) const
			{
				cv::circle(m, (*this)[v].getCenter(), 15, color, -1);
			}

			void drawSpecial(cv::Mat& m) const
			{

				cv::Scalar color_link;
				if(m.channels() == 1){
					color_link = 190;
				}
				else if(m.channels() == 3){
					color_link[0] = 0;
					color_link[1] = 0;
					color_link[2] = 255;
				}
				cv::RNG rrng(12345);


				//first is beginning, second is "past the end"
				std::pair<VertexIteratorLaplacian, VertexIteratorLaplacian> vp;
				//vertices access all the vertix
				for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {

					cv::Scalar color_all_linked;

					if(m.channels() == 1){
						color_all_linked = rrng.uniform(50, 255);
					}
					else if(m.channels() == 3){
						color_all_linked[1] = rrng.uniform(50, 255);
						color_all_linked[2] = rrng.uniform(50, 255);
						color_all_linked[3] = rrng.uniform(50, 255);
					}

					VertexLaplacian v = *vp.first;
					drawSpecial(m, v, color_all_linked);

					double value = 0;
					if((*this)[v].useHeatAnchors() ){
						value = (*this)[v].getHeatAnchors();
					}
					else{
						value = (*this)[v].getHeat();
					}
					(*this)[v].zone.drawZone(m, cv::Scalar(value * 255) );


					EdgeIteratorLaplacian out_i, out_end;
					EdgeLaplacian e;

					for (boost::tie(out_i, out_end) = boost::out_edges(v, (*this));
					     out_i != out_end; ++out_i) {
						e = *out_i;
						VertexLaplacian src = boost::source(e, (*this)), targ = boost::target(e, (*this));
						cv::line(m, (*this)[src].getCenter(), (*this)[targ].getCenter(), color_link, 5);

					}

				}
			}


			double angle(const GraphLaplacian::VertexLaplacian& center, const GraphLaplacian::VertexLaplacian& v1, const GraphLaplacian::VertexLaplacian& v2) const
			{
				double x1 = (*this)[v1].getCenter().x - (*this)[center].getCenter().x;
				double y1 = (*this)[v1].getCenter().y - (*this)[center].getCenter().y;

				double x2 = (*this)[v2].getCenter().x - (*this)[center].getCenter().x;
				double y2 = (*this)[v2].getCenter().y - (*this)[center].getCenter().y;

				// 		std::cout << "values " <<x1 << " " << y1 << " and " << x2 << " " << y2 <<std::endl;

				double angle = atan2(y2, x2) - atan2(y1, x1);
				if(angle < 0){
					angle = angle + (2 * M_PI);
				}
				return angle;

			}

			void getAllEdgeLinkedCounterClockWise(const GraphLaplacian::VertexLaplacian& v, std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > >& all_edge) const
			{

				getAllEdgeLinked(v, all_edge);
				// 		std::cout << "Size in function " << all_edge.size() << std::endl;
				if(all_edge.size() > 0){

					GraphLaplacian::VertexLaplacian firstvertex = all_edge[0].second;

					//classify them in a clock wise manner
					std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > >::iterator it;
					std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > copy;
					std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > >::iterator it_2;
					for(it = all_edge.begin()+1 ; it != all_edge.end() ; it++){

						it_2 = it ;
						copy = *it;

						double angle_to_compare = angle(v, firstvertex, (*it).second);

						while( it_2 != all_edge.begin() && angle(v, firstvertex, (*( it_2-1 )).second) > angle_to_compare){
							*(it_2) = *(it_2-1);
							it_2 = it_2 - 1;
						}
						*(it_2) = copy;

					}


				}

			}

			void getAllVertexAttrCounterClockWise(const GraphLaplacian::VertexLaplacian& v, std::deque< Region>& out) const {
				std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > > all_edge;
				out.clear();

				getAllEdgeLinkedCounterClockWise(v, all_edge);

				for(auto it = all_edge.begin() ; it != all_edge.end() ; ++it){
					out.push_back( (*this)[it->second] );
				}
			}

			void labelAll(const std::deque< GraphLaplacian::VertexLaplacian >& h)
			{

				// 		std::cout << "SIZE " << h.size() << std::endl;
				for(size_t i = 0; i < h.size() ; i++){
					// 			std::cout << "LABELLED" << std::endl;
					(*this)[h[i]].label = true;
				}
			}

			void resetLabel()
			{

				// 		//first is beginning, second is "past the end"
				std::pair<GraphLaplacian::VertexIteratorLaplacian, GraphLaplacian::VertexIteratorLaplacian> vp;
				//vertices access all the vertix
				for (vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
					GraphLaplacian::VertexLaplacian v = *vp.first;
					(*this)[v].label = false;
				}
			}


			void pairWiseMatch(
					const graphmatch::GraphLaplacian& gp2,
					std::deque<graphmatch::MatchLaplacian >& places_pair);

			int editDistance(const GraphLaplacian::VertexLaplacian& v, const std::deque< Region>& other_graph_neighbor, const std::deque< std::pair< GraphLaplacian::EdgeLaplacian, GraphLaplacian::VertexLaplacian > >& all_edge_other_graph, std::deque< graphmatch::MatchLaplacian >& out, std::string& operation_out) const;

			double makeMatching(const AASS::graphmatch::GraphLaplacian::VertexLaplacian& v, const AASS::graphmatch::GraphLaplacian::VertexLaplacian& v_model, const AASS::graphmatch::GraphLaplacian& gp_model, const std::deque< AASS::graphmatch::MatchLaplacian >& matched_previously, std::deque< AASS::graphmatch::MatchLaplacian >& out_match);

			void getNeighborBetween2(size_t start, size_t end, const std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > >& all_edge, std::deque< AASS::graphmatch::GraphLaplacian::VertexLaplacian >& neighbor, std::deque< AASS::graphmatch::Region >& places) const;

			void createMatch(const std::string& operation, const std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > >& all_edge, const std::deque< std::pair< AASS::graphmatch::GraphLaplacian::EdgeLaplacian, AASS::graphmatch::GraphLaplacian::VertexLaplacian > >& all_edge_other_graph, std::pair< int, int > start, std::deque< graphmatch::MatchLaplacian >& out);


		};

	}
}

#endif