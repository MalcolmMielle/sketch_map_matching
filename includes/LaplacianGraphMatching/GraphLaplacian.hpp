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

        class HeatPair{
        public:
            HeatPair(double heatt, double heat_anchort) : heat(heatt), heat_anchor(heat_anchort){}
            double heat = -1;
            double heat_anchor = -1;
            
            double getHeat() const {return heat;}
            double getHeatAnchor() const {return heat_anchor;}
        };

		class Region {
		protected:

			std::vector< std::vector< cv::Point > > _contour;
//			std::deque <cv::Point2i> _zone;
//			cv::Moments moment;
			cv::Point2f _center;

			bool _use_heat_anchors = true;

			double _value_vertex = -1;

			std::map<double, HeatPair > _heats;
// 			double _time = -1;
// 			double _heat_anchors = -1;
            
			double _threshold_same = 0.05;

//			double _eigenvalue;
//			Eigen::VectorXd _eigenvector;

		public:
			bool label = false;


			///JUST FOR TESTING OF ALL METHOD
			std::string type_old_method_testing = "nan";
			bool use_old_method_testing = false;

			AASS::RSI::ZoneRI zone;


			EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
			int index = -1;
			Region(){}

			Region(const Region& r){
				_contour = r.getContour();
				_center = r.getCenter();
				_value_vertex = r.getValue();
// 				_heat = r.getHeat();
                _heats = r.getHeats();
// 				_time = r.getTime();
// 				_heat_anchors = r.getHeatAnchors();
				zone = r.zone;
				type_old_method_testing = r.type_old_method_testing;
				use_old_method_testing = r.use_old_method_testing;
				label = r.label;
				_threshold_same = r.getThresholdSame();
			}

			double getThresholdSame() const {return _threshold_same;}
			void setThresholdSame(double tt) {_threshold_same = tt;}
			void useHeatAnchors(bool b){_use_heat_anchors = b;}
			bool useHeatAnchors() const {return _use_heat_anchors;}

			void print() const {
				std::cout << "Node " << index << std::endl;
                for (auto el : _heats){
                    std::cout << "Time " << el.first << " : heat -> " << el.second.getHeat() << " heat anchor " << el.second.getHeatAnchor() << std::endl;
                }
			}

			void setCenter(const cv::Point2f& center){_center = center;}
			void setContour(const std::vector< std::vector< cv::Point > >& contour){_contour = contour;}

			const cv::Point2f& getCenter() const {return _center;}
			const std::vector< std::vector< cv::Point > >& getContour() const {return _contour;}

			double getValue() const {return _value_vertex;}
			void setValue(double se){_value_vertex = se;}

			std::map<double, HeatPair> getHeats() {return _heats;}
			const std::map<double, HeatPair>& getHeats() const {return _heats;}
// 			double getHeat() const {return _heat;}
// 			double getHeatAnchors() const {return _heat_anchors;}
// 			double getTime() const {return _time;}


            void heatKernel(const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time_from, double time_to, double time_step){
                
                _heats.clear();
                //Just making sure we do it once it case someone says time_step = 0
                if(time_from == time_to){
                    time_step = 1;
                }
                for(double i = time_from; i <= time_to; i = i + time_step){
//                     std::cout << " doing time " << i << std::endl;
                    double score = heatKernel(eigenvalues, eigenvectors, i);
                    if(_heats.find( i ) != _heats.end() ) {
                        std::cout << "time " << i << " with " << time_from << " " << time_to << " " << time_step << std::endl;
                        throw std::runtime_error("Time added twice");
                    }
                    else{
                        HeatPair hp(score, -1);
                        _heats.insert(std::pair<double, HeatPair>(i, hp));
                    }
                }
            }


			
			
			void heatKernelAnchors(const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time_from, double time_to, double time_step, std::deque<int> indexes_anchor){
                
                _heats.clear();
                if(time_from == time_to){
                    time_step = 1;
                }
                for(double i = time_from; i <= time_to; i = i + time_step){
                    
//                     double i_log_scale = std::log10(i - time_from + 1);
                    double i_log_scale = i;
                    
                    double score = heatKernel(eigenvalues, eigenvectors, i_log_scale);
                    double score_anchors = heatKernelAnchors(eigenvalues, eigenvectors, i_log_scale, indexes_anchor);
                    
                    if(_heats.find( i ) != _heats.end() ) {
                        std::cout << "time " << i_log_scale << " with " << time_from << " " << time_to << " " << time_step << std::endl;
                        throw std::runtime_error("Time added twice");
                    }
                    else{
                        HeatPair hp(score, score_anchors);
                        _heats.insert(std::pair<double, HeatPair>(i_log_scale, hp));
                    }
                }
                
//                 std::cout << time_to << " - " << time_from  << " " << " / " << time_step << " = " << (time_to  - time_from) / time_step << " == " << _heats.size() << std::endl;
// 				assert( (time_to - time_from) / time_step == _heats.size() - 1 );
            }

			

			double compare(const Region& region) const {

				if(use_old_method_testing == false) {

// 					assert(region.type_old_method_testing.compare("nan") == 0);
// 					assert(region.getTime() == _time);
// 					assert(_heat_anchors != -1);

//				std::cout << "Heats : " << region.getHeatAnchors() << " - " << _heat_anchors << std::endl;
//				return std::abs( region.getHeatAnchors() - _heat_anchors );
                    //TODO
// 					if (_use_heat_anchors) {
// // 						return std::abs(region.getHeatAnchors() - _heat_anchors);
// 					} else {
// // 						return std::abs(region.getHeat() - _heat);
// 					}
                    return diffHeat(region);
				}
				else{

					assert(region.type_old_method_testing.compare("nan") != 0);
					if(region.type_old_method_testing.compare(type_old_method_testing) == 0){
						return true;
					}
					return false;
				}
			}

//			void setEigen(double value, const Eigen::VectorXd& vector){
//				_eigenvalue = value;
//				_eigenvector = vector;
//			}

//			double getEigenValue(){return _eigenvalue;}
//			const Eigen::VectorXd& getEigenVector() const {return _eigenvector;}


			bool compareBool(const Region& r) const {

//				assert(type_old_method_testing.compare("nan") == 0);
//				assert(r.type_old_method_testing.compare("nan") == 0);

				if(use_old_method_testing == false) {
                    
                    assert(type_old_method_testing.compare("nan") != 0);
                    assert(r.type_old_method_testing.compare("nan") != 0);
// 					assert(r.getTime() == _time);
// 					assert(_heat_anchors != -1);

                    double diff = diffHeat(r);
                    if(diff <= _threshold_same){
                        return true;
                    }
//                     else if(r.type_old_method_testing.compare(type_old_method_testing) == 0){
// 						return true;
// 					}
					return false;
                    
// 					if (_use_heat_anchors) {
//                         //TODO
// //					std::cout << "HEAT " << r.getHeatAnchors() << " " << _heat_anchors << std::endl;
// // 						if (r.getHeatAnchors() <= _heat_anchors + _threshold_same &&
// // 						    r.getHeatAnchors() >= _heat_anchors - _threshold_same) {
// //						std::cout << "True" << std::endl;
// // 							return true;
// // 						}
// 					} else {
// // 						if (r.getHeat() <= _heat + _threshold_same && r.getHeat() >= _heat - _threshold_same) {
// //						std::cout << "True" << std::endl;
// // 							return true;
// // 						}
// 					}
//				std::cout << "False" << std::endl;
// 					return false;
				}
				else{
					assert(r.type_old_method_testing.compare("nan") != 0);
					if(r.type_old_method_testing.compare(type_old_method_testing) == 0){
						return true;
					}
					return false;
				}

			}
			
        private:
            double diffHeat(const Region& r) const {
                
                double diff = 0;
                for(auto el : _heats){
                    double time = el.first;
                    auto heatpair_in_r = r.getHeats().find(time);
                    assert( heatpair_in_r != r.getHeats().end() );
                    if(!_use_heat_anchors){
                        diff = diff + std::abs(heatpair_in_r->second.getHeat() - el.second.getHeat());
                    }
                    else{
                        diff = diff + std::abs(heatpair_in_r->second.getHeatAnchor() - el.second.getHeatAnchor());
                    }
                }
                assert(diff >= 0);
//                 std::cout << "Diff " << diff << " / " << (double) _heats.size() << " == " << diff / (double) _heats.size() << std::endl;
                assert( (diff / (double) _heats.size() ) <= 1.1);
                return diff / (double) _heats.size();
                
            }
            
            double heatKernelAnchor( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time, double index_anchor) const {
				double score = 0;
				for (int i = 0; i < eigenvalues.size(); ++i){
					double eigenvalue = eigenvalues[i];
					double eigenvectorvalue = eigenvectors.col(i)(index);
					double eigenvectorvalue_anchor = eigenvectors.col(i)(index_anchor);
					score = score + std::exp(-time * eigenvalue) * (eigenvectorvalue * eigenvectorvalue_anchor);
				}
				return score;
			}

			double heatKernelAnchors( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time, std::deque<int> indexes_anchor) const {
				double score_anchors = 0;
				for(auto index : indexes_anchor) {
					score_anchors = score_anchors + heatKernelAnchor(eigenvalues, eigenvectors, time, index);
				}
// 				_heat_anchors = score_anchors;
// 				heatKernel(eigenvalues, eigenvectors, time);
				return score_anchors;
			}
			
			double heatKernel( const Eigen::VectorXd& eigenvalues, const Eigen::MatrixXd& eigenvectors, double time) const {
				double score = 0;
				for (int i = 0; i < eigenvalues.size(); ++i){
					double eigenvalue = eigenvalues[i];
					double eigenvectorvalue = eigenvectors.col(i)(index);
					score = score + std::exp(-time * eigenvalue) * (eigenvectorvalue * eigenvectorvalue);
				}

// 				_heat = score;
// 				_time = time;
 				return score;
			}



		};


		inline std::ostream& operator<<(std::ostream& in, const Region &p){

			in << p.getCenter();
			return in;

		}

// 		inline bool operator==(const Region& in, const Region &p){

            //TODO
// 			return in.getHeatAnchors() == p.getHeatAnchors();

// 		}

		inline bool compareRegion(const Region& p, const Region& p2){
			assert(p.use_old_method_testing == false);
			assert(p2.use_old_method_testing == false);
//			assert(false);
			return p.compareBool(p2);
		}

		inline bool compareRegionOldStrategy(const Region& p, const Region& p2){
//			std::cout << "Wait what ? " <<p.type_old_method_testing << std::endl;
			assert(p.type_old_method_testing.compare("nan") != 0);
			assert(p2.type_old_method_testing.compare("nan") != 0);
			assert(p.use_old_method_testing == true);
			assert(p2.use_old_method_testing == true);
			if(p.type_old_method_testing.compare(p2.type_old_method_testing) == 0){
				return true;
			}
			return false;
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

			bool _use_old_comparison_method;
            
            double check_threshold_test = 0.05;

		public:


			GraphLaplacian() {}
			GraphLaplacian( const GraphLaplacian& ) = delete; // non construction-copyable
			GraphLaplacian& operator=( const GraphLaplacian& ) = delete; // non copyable


			void setThrehsoldSameVertices(double n_t){
                
                check_threshold_test = n_t;
				for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
					auto v = *vp.first;
					(*this)[v].setThresholdSame(n_t);
				}
			}

			
			bool test_check_threshold() const {
                for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
					auto v = *vp.first;
					assert( (*this)[v].getThresholdSame() == check_threshold_test);
				}
				return true;
                
            }
			
//			double getThrehsoldSame(){
//				double thres = -1;
//				for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
//					auto v = *vp.first;
//					if(thres == -1) {
//						thres = (*this)[v].getThresholdSame();
//					}
//					if(thres != (*this)[v].getThresholdSame()
//					std::runtime_error("Theshold values differs");
//				}
//			}

			void useOldComparisonMethod(bool b){

				_use_old_comparison_method = b;
                for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
                    auto v = *vp.first;
                    int num_edges = this->getNumEdges(v);

                    //Crossings
                    if (num_edges > 1) {
                        std::cout << "Changing TYPE" << std::endl;
                        (*this)[v].type_old_method_testing = "c";
                    }
                        //Dead ends
                    else {
                        std::cout << "Changing TYPE" << std::endl;
                        (*this)[v].type_old_method_testing = "d";
                    }
                    (*this)[v].use_old_method_testing = b;
                }
                for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
                    auto v = *vp.first;
                    assert((*this)[v].use_old_method_testing == b);
                }
                std::cout << "Check passed " << std::endl;
				

			}

			bool isUsingOldMethod() const {
				if(_use_old_comparison_method == true) {
					for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
						auto v = *vp.first;
						assert((*this)[v].type_old_method_testing.compare("nan") != 0);
						assert((*this)[v].use_old_method_testing);
					}
				}
				std::cout << "Check passed " << std::endl;
				return _use_old_comparison_method;
			}




			//***** DISTANCES *****//


			double getEMD(const GraphLaplacian& graph_model) {
				auto hist = getHistogramEdges();
				auto hist_model = graph_model.getHistogramEdges();
//				cv::normalize(hist, hist);
//				cv::normalize(hist_model, hist_model);
				//make signature
				cv::Mat sig1(hist.rows, 2, CV_32FC1);
				cv::Mat sig2(hist_model.rows, 2, CV_32FC1);

				//fill value into signature
				for (int i=0; i< hist.rows; i++) {

					float binval = hist.at< float>(i);
					sig1.at< float>( i, 0) = binval;
					sig1.at< float>( i, 1) = i;

					binval = hist_model.at< float>(i);
					sig2.at< float>( i, 0) = binval;
					sig2.at< float>( i, 1) = i;

				}

				std::cout << "SIG1 " << sig1 << std::endl;
				std::cout << "SIG2 " << sig2 << std::endl;


				cv::EMD(sig1, sig2, CV_DIST_L2);

			}

			//Better Mahalanobis https://stackoverflow.com/questions/18083486/compare-histograms-in-opencv-and-normalize-similarity-index
			double getBhattacharyyaDistance(const GraphLaplacian& graph_model){
				auto hist = getHistogramEdges();
				auto hist_model = graph_model.getHistogramEdges();
//				cv::normalize(hist, hist);
//				cv::normalize(hist_model, hist_model);
				//"Normalizing"
				double min_h, max_h;
				cv::minMaxLoc(hist, &min_h, &max_h);
				double min_model, max_model;
				cv::minMaxLoc(hist_model, &min_model, &max_model);
				double max = std::max(max_h, max_model);
				hist = hist / max;
				hist_model = hist_model / max;

				double l = cv::compareHist(hist, hist_model, CV_COMP_BHATTACHARYYA);
				assert(l >= 0);
				assert(l <= 1);
				std::cout << "Bah " << l << std::endl;
				return l;
			}

			double getMahalanobisLikelihood(const GraphLaplacian& graph_model) {

				double dist = getMahalanobisDistance(graph_model);
				cv::Mat similarity = getSimilarity(graph_model);

				double factor = 1 / std::sqrt(2 * 3.1415 * cv::determinant(similarity) ) ;
				double l =  factor * std::exp(-0.5 * dist);

//				std::cout << "Likelihood " << factor << " " << std::exp(-0.5 * dist) << " " << l << " det " << cv::determinant(similarity) << std::endl;

				assert(l >= 0);
//				assert(l <= 1);

				return l;

			}

			double getMahalanobisDistance(const GraphLaplacian& graph_model){
				auto hist = getHistogramEdges();
				auto hist_model = graph_model.getHistogramEdges();
				cv::Mat similarity = getSimilarity(hist, hist_model);
				cv::Mat dst = hist - hist_model;
//				std::cout << "dst" << dst << std::endl;
//				std::cout << "dst" << similarity << std::endl;
				cv::MatExpr res = (dst.t() * similarity * dst);
				cv::Mat res_mat = res;
				if (res_mat.rows != 1 || res_mat.cols != 1) throw "Matrix is not 1 by 1!";
				return std::sqrt( res_mat.at<float>(0) );

			}

			cv::Mat getSimilarity(const GraphLaplacian& graph_model){
				auto hist = getHistogramEdges();
				auto hist_model = graph_model.getHistogramEdges();
				return getSimilarity(hist, hist_model);

			}


			cv::Mat getSimilarity(cv::Mat& hist, cv::Mat& hist_model){

				cv::Mat similarity(hist.rows, hist_model.rows, hist.type());

//				std::cout << "S rows " << hist.rows << " cols " << hist.cols << " sim " << similarity << std::endl;
				for (int i=0; i< hist.rows; i++) {
					for (int j=0; j< hist_model.rows; j++) {
//						std::cout <<  hist.at<float>(i) << "-" << hist_model.at<float>(j) << " = " << hist.at<float>(i) - hist_model.at<float>(j) << " ans " << std::abs( hist.at<float>(i) - hist_model.at<float>(j) ) << " at " << i << " " << j << std::endl;

//i is row and j is col. is it correct ?
						similarity.at<float>(i, j) = std::abs( hist.at<float>(i) - hist_model.at<float>(j) );

//						std::cout << "sim " << similarity << std::endl;
					}
				}
				return similarity;
			}

			double getChiSquare(const GraphLaplacian& graph_model){
				auto hist = getHistogramEdges();
				auto hist_model = graph_model.getHistogramEdges();
//				cv::normalize(hist, hist);
//				cv::normalize(hist_model, hist_model);

				//"Normalizing"
				double min_h, max_h;
				cv::minMaxLoc(hist, &min_h, &max_h);
				double min_model, max_model;
				cv::minMaxLoc(hist_model, &min_model, &max_model);
				double max = std::max(max_h, max_model);
				hist = hist / max;
				hist_model = hist_model / max;

				double l = cv::compareHist(hist, hist_model, CV_COMP_CHISQR);
				assert(l >= 0);
				assert(l <= 1);
				std::cout << "Chi " << l << std::endl;
				return l;
			}


			cv::Mat getHistogramEdges() const {

				std::vector<int> edges;
				for (auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first) {
					auto v = *vp.first;
					edges.push_back( getNumEdges(v) );
				}
				cv::Mat cvt(edges, false);
				cvt.convertTo( cvt, CV_8UC1 );

//				std::cout << "Edges " << cvt << "\n DONE" << std::endl;

				cv::Mat hist;
				/// Establish the number of bins
				int histSize = 5;
				/// Set the ranges ( for B,G,R) )
				float range[] = { 0, histSize } ;
				const float* histRange = { range };
				bool uniform = true; bool accumulate = false;
				cv::calcHist(&cvt, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

//				cv::normalize(hist, hist);

				std::cout << "Histogram " << hist << std::endl;
				return hist;
			}



			double getMeanL2Norm(const GraphLaplacian& graph) const {
				if(this->getNumVertices() <= graph.getNumVertices()){
					return this->getMeanL2NormOneWay(graph);
				}
				else{
					return graph.getMeanL2NormOneWay(*this);
				}
			}

			double getMeanL2NormOneWay(const GraphLaplacian& graph) const {

//				assert(_size_classified == true);
				double sum_distance = 0;

				for(auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first){
					VertexLaplacian v = *vp.first;
					double shortest_distance_relative_size = -1;
					double size_classification = (*this)[v].zone.getSizeClassification();

					for(auto vp_model = boost::vertices(graph); vp_model.first != vp_model.second; ++vp_model.first){
						VertexLaplacian v_model = *vp_model.first;
						double size_classification_model = graph[v_model].zone.getSizeClassification();
						double distance = std::abs(size_classification -  size_classification_model);
						if(shortest_distance_relative_size == -1 || shortest_distance_relative_size > distance){
							shortest_distance_relative_size = distance;
						}
					}

					sum_distance += shortest_distance_relative_size;

				}
				return sum_distance / (double) this->getNumVertices();

			}

			double getHausdorffDistanceRelativeSize(const GraphLaplacian& graph) const {

				double shortest_this_to_graph = this->getLongestDistanceOutOfShortestBetweenRegionSize(graph);
				double shortest_graph_to_this = graph.getLongestDistanceOutOfShortestBetweenRegionSize(*this);
				return std::max(shortest_graph_to_this, shortest_this_to_graph);

			}

			double getLongestDistanceOutOfShortestBetweenRegionSize(const GraphLaplacian& graph) const{

//				assert(_size_classified == true);
				double longest_distance_relative_size = -1;

				for(auto vp = boost::vertices((*this)); vp.first != vp.second; ++vp.first){
					VertexLaplacian v = *vp.first;
					double shortest_distance_relative_size = -1;
					double size_classification = (*this)[v].zone.getSizeClassification();

					for(auto vp_model = boost::vertices(graph); vp_model.first != vp_model.second; ++vp_model.first){
						VertexLaplacian v_model = *vp_model.first;
						double size_classification_model = graph[v_model].zone.getSizeClassification();
						double distance = std::abs(size_classification -  size_classification_model);
						if(shortest_distance_relative_size == -1 || shortest_distance_relative_size > distance){
							shortest_distance_relative_size = distance;
						}
					}

					if(shortest_distance_relative_size > longest_distance_relative_size){
						longest_distance_relative_size = shortest_distance_relative_size;
					}

				}
				return longest_distance_relative_size;
			}


			/***********************************************************/





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
            std::deque<VertexLaplacian>& getAnchors(){return _anchors;}
            const std::deque<VertexLaplacian>& getAnchors() const {return _anchors;}

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

			/**
			 * @param[in] factor : factor used to "normalize" the weights. Typically, the hausdorff distance or the l2 norm
			 */
			void useRelativeSizeAsWeights(double factor = 1){
//				std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp3;

				std::cout << "Factor " << factor << std::endl;
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

				double max_val = 0;
				for (vp3 = boost::vertices(*this); vp3.first != vp3.second; ++vp3.first) {
					v = *vp3.first;
					double size_class = (*this)[v].zone.getSizeClassification();

					double k = 1;
//					double value_region = ( 1 / factor ) +  (factor * (size_class - lowest_value) );
					double value_region = (k * factor) + ( (size_class - lowest_value) * (1 - (k * factor)) );
					if(max_val < value_region){
						max_val = value_region;
					}
//					std::cout << "From " << size_class - lowest_value << " to " <<value_region << std::endl;
//					(*this)[v].setValue(value_region);
				}

				//Equalizing to 1
//				for (vp3 = boost::vertices(*this); vp3.first != vp3.second; ++vp3.first) {
//					v = *vp3.first;
//					double value = (*this)[v].getValue();
//					std::cout << "Final value " << value << " TO " << value / max_val << std::endl;
//					(*this)[v].setValue(value / max_val);
//				}
//				int aaaa;
//				std::cin >> aaaa;
			}

			/**
			 *
			 * @return the adjancy matrix with the sources as the rows and the target as columns
			 */
			Eigen::MatrixXd getAdjancyMatrix();
			Eigen::MatrixXd getWeightedGeneralizedLaplacian();

			std::tuple<Eigen::VectorXd, Eigen::MatrixXd> eigenLaplacian();

			void laplacianFamilySignatureGeneration(){};

			void printHeatKernelValueNode(const VertexLaplacian& vertex) const{
				(*this)[vertex].print();
			}

			void propagateHeatKernel(double time_from, double time_to, double time_step){

				std::deque<int> index_anchors;
				for(auto anchor : _anchors){
					index_anchors.push_back( (*this)[anchor].index );
				}

				std::pair<VertexIteratorLaplacian, VertexIteratorLaplacian> vp;
				for (vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
					VertexLaplacian vertex_in = *vp.first;
					(*this)[vertex_in].heatKernelAnchors(_eigenvalues, _eigenvectors, time_from, time_to, time_step, index_anchors);
				}
				
			}

			std::vector<AASS::graphmatch::MatchLaplacian> compare(const GraphLaplacian& gl) const ;

			HypotheseLaplacian hungarian_matching(const AASS::graphmatch::GraphLaplacian& laplacian_model);


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
                    //TODO
					if((*this)[v].useHeatAnchors() ){
// 						value = (*this)[v].getHeatAnchors();
					}
					else{
// 						value = (*this)[v].getHeat();
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

            
            AASS::graphmatch::GraphLaplacian::VertexLaplacian getVertex(const cv::Point2i& map_point) const {
                    
                    
                    AASS::graphmatch::GraphLaplacian::VertexLaplacian v_closest_if_not_found;
                    double smallest_dist = -1;
                    
                    for(auto vp = boost::vertices(*this); vp.first != vp.second; ++vp.first) {
                        AASS::graphmatch::GraphLaplacian::VertexLaplacian v = *vp.first;
                        auto zone = (*this)[v].zone.getZone();
                        
//                         cv::Scalar scal(255);
//                         cv::Scalar color(150);
//                         cv::Mat zone_img = cv::Mat::zeros(500, 500, CV_8UC1);
//                         (graph)[v].drawZone(zone_img, scal);
//                         cv::circle(zone_img, cv::Point2i(map_point.y, map_point.x), 5, color, -1);
//                         cv::imshow ("zone test", zone_img);
//                         cv::waitKey(0);
                        
                        
//                         auto region = graph[v];
//                         auto zone = region.getZone();
                        for(auto point : zone ){
                            
                            double dist = cv::norm(map_point - point);
                            if(map_point == point){
//                                 std::cout << "FOUND: " << v << std::endl;
                                return v;
                            }
                            else{
                                if(dist <= smallest_dist || smallest_dist == -1){
                                    smallest_dist = dist;
                                    v_closest_if_not_found = v;
                                }
                            }
                        }
                    }
                    
//                     std::cout << map_point << " at point " << std::endl;
                    if(smallest_dist <= 10){
                        return v_closest_if_not_found;
                    }
                    std::cout << "Smallest distance " << smallest_dist << std::endl;
                    throw std::runtime_error("Zone not found");
// 					std::cout << "Not found" << std::endl;
				}
				

		};

	}
}

#endif
