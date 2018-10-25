#ifndef COMPARATORANCHOR_MAP
#define COMPARATORANCHOR_MAP

#include "GraphMatcherNeighbor.hpp"
#include "MatchingFunctions.hpp"
#include <boost/format.hpp>

//FOR TESTING 
#ifdef DEBUG
#include "Util.hpp"
#endif

#ifdef TIMED
#include "Timed.hpp"
#endif


namespace AASS{
		
	namespace graphmatch{
		
		/**
		 * @brief Anchor Matching
		 * 
		 * Detect all anchor
		 * 
		 * Expend algo of neighbor matching for all doors
		 * 
		 * Fuse all best match by comparing cost for evey anchor matching possible by comparing the cost
		 * 
		 * Return final matching
		 * 
		 */
		class GraphMatcherAnchor : public AASS::graphmatch::GraphMatcherBase{
		protected :
			
			std::deque < graphmatch::Match > _anchors;
			std::deque < graphmatch::Hypothese > _allhypothese_from_each_anchor;
			GraphMatcherNeighbor _gm_neighbor;


			bool _use_lowest_value_for_matching = true;
			bool _use_zone_of_equal_value_for_matching = false;
			bool _keep_best_anchor = false;
			
		private:
			std::deque< int > _from_which_hypo;
			int _studied_index_to_fuse;
			bool _draw;
			
		public :
			GraphMatcherAnchor() : GraphMatcherBase(), _draw(0){};

			void useLowestValueForMatching(bool u){_use_lowest_value_for_matching = u;}
			void useZoneOfEqualValueForMatching(bool u){_use_zone_of_equal_value_for_matching = u;}
			void useBestAnchor(bool u){_keep_best_anchor = u;}
			
			/**
			 * @brief Main matching algorithm nusing anchors
			 *
			 *The template is just here for compatibility with the upper probabilistic algorithm 
			 * 
			 * */
			bool anchorMatching(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, std::deque < graphmatch::Match >& anchors, bool draw, cv::Size size = cv::Size(300,300) );
			bool anchorMatching(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, std::deque < graphmatch::Match >& anchors){
				return anchorMatching(gp, gp_model, anchors , 0);
			}
			
			void pushAnchor(const Match& m){_anchors.push_back(m);}
			
			virtual bool match(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model)
			{
				return anchorMatching(gp, gp_model, _anchors);
			};
			
			virtual void clear(){
// 				std::cout << "Clear the anchors" << std::endl;
				GraphMatcherBase::clear();
				_gm_neighbor.clear();
				_allhypothese_from_each_anchor.clear();
				_anchors.clear();
			}
			
		private:


			void getAnchorsMatching(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model, std::deque < graphmatch::Match >& anchors, bool sort_by_score = true);



//			/**
//			* @brief compare the two first match to the last one and return true if the cost of the last one is less than both cost of the first ones.
//			*
//			* ATTENTION : If one want to compare only two matches, then into the same Match for match_original and match_maybe
//			*
//			* @param[in] match_original : Match to compare
//			* @param[in] match_maybe : a maybe second match to compare. input the same Match as for match_original to not use.
//			* @param[in] match_to_compare : the Match we want to know if the cost is less than match_original and match_maybe
//			*
//			* @return true if match_to_compare cost is less than match_original and match_maybe, false otherwise.
//			*/
//			bool bestMatch(const AASS::graphmatch::Match& match_original, const AASS::graphmatch::Match& match_maybe, const AASS::graphmatch::Match& match_to_compare, size_t& diff) const;
			
			/**
			* @brief compare the two first match to the last one and either keep the two first match, or remove them and add the new match to final (while removing the new match from the list of potential match)
			* 
			* ATTENTION : If one want to compare only two matches, then into the same Match for match_original and match_maybe
			* 
			* @param[in] gp : GraphPlace input
			* @param[in] gp_model : GraphPlace model
			* @param[in] final : final Hypothesis.
			* @param[in] match_original : Match to compare.
			* @param[in] index_match_original : index of match_original in final.
			* @param[in] match_maybe : a maybe second match to compare. input the same Match as for match_original to not use.
			* @param[in] index_match_original : index of match_maybe in final.
			* @param[in] match_to_compare : the Match we want to know if the cost is less than match_original and match_maybe
			* 
			* @return true if match_to_compare cost is less than match_original and match_maybe, false otherwise.
			*/
			bool checkAndReplace(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final, AASS::graphmatch::Hypothese& to_fuse, const AASS::graphmatch::Match& match_original, int index_match_original, const AASS::graphmatch::Match& match_maybe, int index_match_maybe, const AASS::graphmatch::Match& match_to_compare, int index_match_to_compare, cv::Size size = cv::Size(300,300)); 
			
			bool replace(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final, AASS::graphmatch::Hypothese& to_fuse, const AASS::graphmatch::Match& match_original, int index_match_original, const AASS::graphmatch::Match& match_maybe, int index_match_maybe, const AASS::graphmatch::Match& match_to_compare, int index_match_to_compare);


			void fuse_hypo(AASS::graphmatch::Hypothese& final_hyp, AASS::graphmatch::Hypothese& to_fuse, bool draw = false, cv::Size size = cv::Size(300,300), AASS::graphmatch::GraphPlace gp = AASS::graphmatch::GraphPlace(), AASS::graphmatch::GraphPlace gp_model = AASS::graphmatch::GraphPlace());
			
			void match_one_hypo(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final_hyp, AASS::graphmatch::Hypothese& to_fuse, size_t& k, bool& is_in_final, bool draw, cv::Size size = cv::Size(300,300));
			void match_one_hypo(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final_hyp, AASS::graphmatch::Hypothese& to_fuse, size_t& k, bool& is_in_final){
				match_one_hypo(gp, gp_model, final_hyp, to_fuse, k, is_in_final, 0);
			}
			
			
		};


		inline void AASS::graphmatch::GraphMatcherAnchor::getAnchorsMatching(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model, std::deque < graphmatch::Match >& anchors, bool sort_by_score){

			for(size_t i = 0 ; i < anchors.size() ; i++){
				//Put all doors combination as seeds

				std::cout << "Pushing an anchor " << anchors[i] << std::endl;

				graphmatch::Hypothese starting_seeds;
				starting_seeds.push_back(anchors[i]);
				_gm_neighbor.clear();
				//Expend planar algo for all doors
#ifdef TIMED
				std::ostringstream str_test;
					str_test <<  "planar matching, line" << __LINE__ << " in file " << __FILE__;
					timed(str_test.str(), boost::bind( &graphmatch::GraphMatcherNeighbor::planarEditDistanceAlgorithm, &_gm_neighbor, boost::ref(starting_seeds), boost::ref(gp), boost::ref(gp_model) ) );
#else
				_gm_neighbor.planarEditDistanceAlgorithm(starting_seeds, gp, gp_model);
#endif
				std::deque < Hypothese > res = _gm_neighbor.getResult();
				_gm_neighbor.sort(res);
				//Save the best results.
				_allhypothese_from_each_anchor.push_back( res[0] );

			}

			std::sort(_allhypothese_from_each_anchor.begin(), _allhypothese_from_each_anchor.end(), [](AASS::graphmatch::Hypothese &hyp, AASS::graphmatch::Hypothese &hyp1){
				return hyp.getDist() < hyp1.getDist();
			} );

//			for(auto hy : _allhypothese_from_each_anchor){
//				std::cout << "DIST" << hy.getDist() << std::endl;
//			}

//			exit(0);




		}



		inline bool AASS::graphmatch::GraphMatcherAnchor::anchorMatching(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model, std::deque < graphmatch::Match >& anchors, bool draw, cv::Size size) {
			_draw = draw;
			cv::Mat mat_in;
			if (_draw == 1) {
				mat_in = cv::Mat::zeros(size, CV_8U);
			}
// 			std::cout << "ANCHORS Here :" << anchors.size() << std::endl;

			cv::Mat m2 = cv::Mat::zeros(size, CV_8U);
			gp.draw(m2);
			cv::imshow("gp input", m2);
			cv::Mat m22 = cv::Mat::zeros(size, CV_8U);
			gp_model.draw(m22);
			cv::imshow("gp model", m22);
			cv::waitKey(0);

			//Get all doors
			if (anchors.size() > 0) {

				getAnchorsMatching(gp, gp_model, anchors);

				Hypothese final_hyp = _allhypothese_from_each_anchor[0];
				for(int idx = 1 ; idx < _allhypothese_from_each_anchor.size() ; ++idx) {

					std::cout << "Fusing hypo " << idx << std::endl;
					fuse_hypo(final_hyp, _allhypothese_from_each_anchor[idx], draw, size, gp, gp_model);

				}

				std::cout << "Done fusing" << std::endl;
				std::cout << "OUT AT THE END OF ANCHOR. Size of hypo " << final_hyp.size() << std::endl;
				this->_hypothesis_final.push_back(final_hyp);
			}

			if(this->_hypothesis_final.size() > 0){
				return true;
			}
			else{
				return false;
			}

		}




//
//		inline bool AASS::graphmatch::GraphMatcherAnchor::anchorMatching(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_model, std::deque < graphmatch::Match >& anchors, bool draw, cv::Size size)
//		{
//			_draw = draw;
//			cv::Mat mat_in;
//			if(_draw == 1) {
//				mat_in = cv::Mat::zeros(size, CV_8U);
//			}
//// 			std::cout << "ANCHORS Here :" << anchors.size() << std::endl;
//
//			cv::Mat m2 = cv::Mat::zeros(size, CV_8U);
//			gp.draw(m2);
//			cv::imshow("gp input", m2);
//			cv::Mat m22 = cv::Mat::zeros(size, CV_8U);
//			gp_model.draw(m22);
//			cv::imshow("gp model", m22);
//			cv::waitKey(0);
//
//
//
//
//			//Get all doors
//			if(anchors.size() > 0){
//
//				getAnchorsMatching(gp, gp_model, anchors);
//
////				for(size_t i = 0 ; i < anchors.size() ; i++){
////				//Put all doors combination as seeds
////
////					std::cout << "Pushing an anchor " << anchors[i] << std::endl;
////
////					graphmatch::Hypothese starting_seeds;
////					starting_seeds.push_back(anchors[i]);
////					_gm_neighbor.clear();
////					//Expend planar algo for all doors
////#ifdef TIMED
////					std::ostringstream str_test;
////					str_test <<  "planar matching, line" << __LINE__ << " in file " << __FILE__;
////					timed(str_test.str(), boost::bind( &graphmatch::GraphMatcherNeighbor::planarEditDistanceAlgorithm, &_gm_neighbor, boost::ref(starting_seeds), boost::ref(gp), boost::ref(gp_model) ) );
////#else
////					_gm_neighbor.planarEditDistanceAlgorithm(starting_seeds, gp, gp_model);
////#endif
////					std::deque < Hypothese > res = _gm_neighbor.getResult();
////					_gm_neighbor.sort(res);
////					//Save the best results.
////					_allhypothese_from_each_anchor.push_back( res[0] );
////
////				}
//
//// 				cv::Mat mat_in = cv::imread("../Test/Sequences/missingmap.png");
//// 				_gm_neighbor.drawHypo(gp, gp_model, mat_in, mat_in, _allhypothese_from_each_anchor[0].getMatches(), "GM MATCH", 2);
//// 				cv::waitKey(0);
//			// 	std::cout << "Size of all hypo and seeds " << _allhypothese_from_each_anchor.size() << std::endl;
//
//				//Fuse best match of all expension
//
//				//Push all matches from first Hypo in -> FusedHypo
//
//				Hypothese final_hyp = _allhypothese_from_each_anchor[0];
//
//				//Test that every vertex exist
//				gp.print();
//				for(size_t i = 0 ; i != final_hyp.size() ; ++i){
//					std::pair<graphmatch::VertexIteratorPlace, graphmatch::VertexIteratorPlace> vp;
//					//vertices access all the vertices
//					bool exist = false;
//					for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; ++vp.first) {
//						graphmatch::VertexPlace v = *vp.first;
//						if(v == final_hyp[i].getFirst()){
//							exist = true;
//							std::cout << "Found" << std::endl;
//						}
//					}
//					std::cout <<"checked " << final_hyp[i].getFirst() << " at " << gp[final_hyp[i].getFirst()].mass_center << "linked to " << gp_model[final_hyp[i].getSecond()].mass_center << std::endl;
//					assert(exist == true);
//				}
//
//				/////////////////////////////////////////////////////////////////////////////
//				/** Add index in _from_which_hypo_to_fuse to remember from which hypo is each match
//				 */
//				_from_which_hypo.clear();
//				_studied_index_to_fuse = 0;
//				for(size_t i = 0 ; i < final_hyp.size() ; i++){
//					_from_which_hypo.push_back(_studied_index_to_fuse);
//				}
//
//				/////////////////////////////////////////////////////////////////////////////
//				/**
//				 * Fusing
//				 */
//
//				for(size_t i = 1 ; i < _allhypothese_from_each_anchor.size() ; i++){
//
//					std::cout << "TESTING NEW HYPO" << std::endl;
//					Hypothese to_fuse = _allhypothese_from_each_anchor[i];
//					_studied_index_to_fuse = i;
//					//For every match,
//					//Check if any of the vertices already exist. If not add.
//					if(_draw == true){
//						final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "final before", 1);
//						to_fuse.drawHypo(gp, gp_model, mat_in, mat_in, "to fuse", 1);
//// 						anchors.drawHypo(gp, gp_model, mat_in, mat_in, "Anchor for fuse", 2);
//						cv::waitKey(0);
//// 						exit(0);
//					}
//
////
//					bool is_in_final = true;
//					//This step is needed to not study the match we add from fuse at the end of the deque
//					size_t size_fuse = to_fuse.size();
//					size_t size_final = final_hyp.size();
//
//
//// 					/****************TEST to rmeove later***************/
//
//				#ifdef DEBUG
//// 					std::cout << "Teseting for double match" << std::endl;
//// 					std::cout << "Testing to fuse : " << to_fuse.size() << std::endl;
//					containsDoubleMatch(to_fuse);
//// 					std::cout << "Testing final : " << final.size() << std::endl;
//					containsDoubleMatch(final_hyp);
//				#endif
//
//// 					/****************************************************/
//
//					for(size_t k = 0 ; k < size_fuse ; k++){
//
//						is_in_final = false;
//
//#ifdef TIMED
//						std::ostringstream str_test;
//						str_test <<  "match one hyp, line" << __LINE__ << " in file " << __FILE__;
//						timed(str_test.str(), boost::bind( &GraphMatcherAnchor::match_one_hypo, this, boost::ref(gp), boost::ref(gp_model), boost::ref(final_hyp), boost::ref(to_fuse), boost::ref(k), boost::ref(is_in_final), draw ) );
//						std::cout << "OUT OF MATCH TIMED" << std::endl;
//
//#else
//// 						std::cout << "IN MATCH ONE NOT TIMED" << std::endl;
//						match_one_hypo(gp, gp_model, final_hyp, to_fuse, k, is_in_final, draw, size);
//// 						std::cout << "OUT OF MATCH ONE" << std::endl;
//#endif
//
//						if(is_in_final == false){
//							std::cout << "NEW ONE" << std::endl;
//							final_hyp.push_back(to_fuse[k]);
//							_from_which_hypo.push_back(_studied_index_to_fuse);
//							++size_final;
//
//							if(_draw == true){
//								Hypothese dm;
//								dm.push_back(to_fuse[k]);
//// 								std::cout << std::endl << "NEWNEWNEW " << std::endl << std::endl;
//								dm.drawHypo(gp, gp_model, mat_in, mat_in, "NEW ONE", 2);
//								cv::waitKey(0);
//							}
//
//						}
//						else{
//// 							std::cout << "WAS SENN BEFORE " << std::endl;
//						}
//
//					}
//
//
//				}
//
//
//				/* Second run :
//				 * The second run is here to make sure no link has ben forgotten.
//				 * Example case.
//				 * Link 1 remove Link 2 and Link 3 and links node 1 and node 3
//				 * Link 2 is linking node1 and node 2
//				 * Link 3 is linking node 3 and node 4
//				 *
//				 * If link for node 2 and node 4 was earlier in the to_fuse list,
//				 * it will never come and the link will disappear
//				 * If we run a second time, the program will realize that
//				 * the link 2 -> 4 can actually be added now and do so
//				 *
//				 * For now it does another full comparison.
//				 * Maybe we should only check for non present link.
//				 *
//				 */
//
//				//Fuse
//				for(size_t i = 1 ; i < _allhypothese_from_each_anchor.size() ; i++){
//
//					Hypothese to_fuse = _allhypothese_from_each_anchor[i];
//					//For every match,
//					//Check if any of the vertices already exist. If not add.
//// 					this->drawHypo(gp, gp_model, mat_in, mat_in, final.getMatches(), "final before", 2);
//// 					this->drawHypo(gp, gp_model, mat_in, mat_in, to_fuse.getMatches(), "to fuse", 2);
//// 					this->drawHypo(gp, gp_model, mat_in, mat_in, anchors, "Anchor for fuse", 2);
//
//// 					cv::waitKey(0);
//
//					bool is_in_final = true;
//
//					//This step is needed to not study the match we add from fuse at the end of the deque
//					size_t size_fuse = to_fuse.size();
////					size_t size_final = final_hyp.size();
//
//
//// 					/****************TEST to rmeove later***************/
//
//				#ifdef DEBUG
//// 					std::cout << "Teseting for double match" << std::endl;
//// 					std::cout << "Testing to fuse : " << to_fuse.size() << std::endl;
//					containsDoubleMatch(to_fuse);
//// 					std::cout << "Testing final : " << final.size() << std::endl;
//					containsDoubleMatch(final_hyp);
//				#endif
//
//// 					/****************************************************/
//
//					for(size_t k = 0 ; k < size_fuse ; k++){
//
//						std::cout << "FUSE...." << std::endl;
//
//						is_in_final = false;
//
//#ifdef TIMED
//						std::ostringstream str_test;
//// 						str_test <<  "match one hyp, line" << __LINE__ << " in file " << __FILE__;
//						timed(str_test.str(), boost::bind( &GraphMatcherAnchor::match_one_hypo, this, boost::ref(gp), boost::ref(gp_model), boost::ref(final_hyp), boost::ref(size_final), boost::ref(to_fuse), boost::ref(k), boost::ref(is_in_final), draw ) );
//// 						std::cout << "OUT OF MATCH TIMED" << std::endl;
//
//#else
//// 						std::cout << "IN MATCH ONE NOT TIMED" << std::endl;
//						match_one_hypo(gp, gp_model, final_hyp, to_fuse, k, is_in_final, draw, size);
//// 						std::cout << "OUT OF MATCH ONE" << std::endl;
//#endif
//						//If it is not part of the hypo then fuse
//						if(is_in_final == false){
//							std::cout << "NEW ONE" << std::endl;
//							final_hyp.push_back(to_fuse[k]);
//							_from_which_hypo.push_back(_studied_index_to_fuse);
//
//							if(_draw == true){
//								Hypothese dm;
//								dm.push_back(to_fuse[k]);
//
//// 								std::cout << std::endl << "NEWNEWNEW " << std::endl << std::endl;
//								dm.drawHypo(gp, gp_model, mat_in, mat_in, "NEW ONE", 1);
//								cv::waitKey(0);
//							}
//
//						}
//						else{
//// 							std::cout << "WAS SENN BEFORE " << std::endl;
//						}
//
//					}
//
//
//				}
//
//				//For all matches in hypo
//					//If both node of match do not exist in
//
//				//Update the distance
//// 				std::cout << "THE ONE UPDATE" << std::endl;
//				int newdist = final_hyp.updateDistance(gp, gp_model);
//				final_hyp.setDist(newdist);
//				//Add solution
//				this->_hypothesis_final.push_back(final_hyp);
//
//			}
//			else{
//
//				std::cout << "normal planar matching because anchor less than or = 0 " << std::endl;
//
//				//Just do a normal planar edit distance
//#ifdef TIMED
//				std::ostringstream str_test;
//				str_test <<  "planar matching, line" << __LINE__ << " in file " << __FILE__;
//				timed(str_test.str(), boost::bind( &graphmatch::GraphMatcherNeighbor::planarEditDistanceAlgorithm, &_gm_neighbor, boost::ref(gp), boost::ref(gp_model) ) );
//#else
//				_gm_neighbor.planarEditDistanceAlgorithm(gp, gp_model);
//#endif
//				std::deque < Hypothese > res = _gm_neighbor.getResult();
//				_gm_neighbor.sort(res);
//				this->_hypothesis_final = res;
//
//			}
//
//			/****************TEST to rmeove later***************/
//
//		#ifdef DEBUG
//// 			std::cout << "Testing for double match at the end" << std::endl;
//// 			std::cout << "Testing final at the end" << std::endl;
//			containsDoubleMatch(this->_hypothesis_final[0]);
//			containsDoubleLink(this->_hypothesis_final[0]);
//		#endif
//
//			/****************************************************/
//			_draw = false;
//
//			if(this->_hypothesis_final.size() > 0){
//				return true;
//			}
//			else{
//				return false;
//			}
//
//
//		}

		
		
		
		inline void GraphMatcherAnchor::match_one_hypo(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final_hyp, AASS::graphmatch::Hypothese& to_fuse, size_t& k, bool& is_in_final, bool draw, cv::Size size)
		{
			
			cv::Mat mat_in = cv::Mat::zeros(size, CV_8U);
			if(draw == true) {
				mat_in = cv::Mat::zeros(size, CV_8U);
			}			
			
#ifdef TIMED
			double time = 0 ;
#endif
			std::cout << "NEW MATCH TO TESt is seen : " << is_in_final << std::endl;


			for(size_t j = 0 ; j < final_hyp.size() ; j++){
				std::cout << " all the number size_final " << final_hyp.size() << " j " << j << " size to_fuse " << to_fuse.size() << " k " << k << std::endl;
				Hypothese dm;
					
				dm.push_back(final_hyp[j]);
				
				dm.drawHypo(gp, gp_model, mat_in, mat_in, "the too fuse we are looking at :D", 1);
				dm.clear();
				
				dm.push_back(to_fuse[k]);
				dm.drawHypo(gp, gp_model, mat_in, mat_in, "2the too fuse we are looking at :D 2", 1);

				/****************TEST to rmeove later***************/
		

#ifdef DEBUG
// 							std::cout << "Testing for double match in loop at " << k << " " << size_fuse << " " <<j << " " << size_final << std::endl;
// 							std::cout << "Testing to fuse : " << to_fuse.size() << std::endl;
				containsDoubleMatch(to_fuse);
				containsDoubleLink(to_fuse);
				
// 							std::cout << "Testing final : " << final.size() << std::endl;
				containsDoubleMatch(final_hyp);
				containsDoubleLink(final_hyp);	
#endif
// 							iu++;
				
				/****************************************************/
				
				//Test the first part of the pair
				std::cout << "TEST " << std::endl;
				Match match;
				Match match_to_compare;
				Match other;
				int index_other = -1;
				if(final_hyp[j].getFirst() == to_fuse[k].getFirst()) {
					std::cout << "SEEN" << std::endl;
					is_in_final = true;
					//Both match to compare
					match = final_hyp[j];
					match_to_compare = to_fuse[k];

					/* We found a common node in the input graph.
					* We need to compare the two match to know if replacing it is an advantage or not
					* But to compare it, maybe, the new match is matched in the model graph to a very good vertex.
					* So we need to compare the new match to the match of the input AND the model in -->final<--.
					* */
					other = match;
//					index_other = -1;
					for (size_t f = 0; f < final_hyp.size(); f++) {
						//Look if the second vertex is the same AND if the match is not the already selected match
						if (to_fuse[k].getSecond() == final_hyp[f].getSecond() && f != j) {
							std::cout << "Found at " << f << std::endl;
							other = final_hyp[f];
							index_other = f;
						}
					}

				}//Test the second part
				else if(final_hyp[j].getSecond() == to_fuse[k].getSecond()) {
					std::cout << "SEEN2" << std::endl;
					is_in_final = true;
					//Both match to compare
					match = final_hyp[j];
					match_to_compare = to_fuse[k];

					/* We found a common node in the input graph.
					* We need to compare the two match to know if replacing it is an advantage or not
					* But to compare it, maybe, the new match is matched in the model graph to a very good vertex.
					* So we need to compare the new match to the match of the input AND the model in -->final<--.
					* */
					other = match;
//					index_other = -1;
					for (size_t f = 0; f < final_hyp.size(); f++) {
						if (to_fuse[k].getFirst() == final_hyp[f].getFirst() && f != j) {
							other = final_hyp[f];
							index_other = f;
						}
					}
				}


				std::cout << "Index other "<< index_other << std::endl;
				//Check if good and then replace.
				if(draw == true) {
					Hypothese dm;
					Hypothese dm3;
					Hypothese dm2;
					Hypothese dmweird;

					dm.push_back(match);
					dm3.push_back(other);
					dm2.push_back(match_to_compare);
// 						dmweird.push_back(final_hyp[9]);
					if (match == other) {
						std::cout << "SAME at " << k << " " << j << std::endl;
					} else {
						std::cout << "Not the same at " << k << " " << j << std::endl;
					}
					dm.drawHypo(gp, gp_model, mat_in, mat_in, "match", 1);
					dm3.drawHypo(gp, gp_model, mat_in, mat_in, "match2", 1);
					dm2.drawHypo(gp, gp_model, mat_in, mat_in, "match to compare", 1);
// 						this->drawHypo(gp, gp_model, mat_in, mat_in, dmweird, "weirdo", 2);

					final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "final just before", 1);
				}
					
#ifdef TIMED
					std::ostringstream str_test;
					str_test <<  "Replace, line" << __LINE__ << " in file " << __FILE__;
// 								timed(str_test.str(), 
// 									
// 								);
					//TOO MANY ARGUMENT :( MAXIMUM IS 9. It is super sad.
// 								boost::bind( 
// 										&graphmatch::GraphMatcherAnchor::replace, 
// 										this,
// 										boost::ref(gp), 
// 										boost::ref(gp_model), 
// 										boost::ref(final_hyp), 
// 										boost::ref(to_fuse), 
// 										boost::cref(match), 
// 										j, 
// 										boost::cref(other), 
// 										index_other, 
// 										boost::cref(match_to_compare), 
// 										k
// 									) ;
						

					std::clock_t a = std::clock();
					bool replaced = checkAndReplace(gp, gp_model, final_hyp, to_fuse, match, j, other, index_other, match_to_compare, k, size);	
					std::clock_t b = std::clock();	

					time = time + ( ((float)(b - a))/CLOCKS_PER_SEC);
					
// 					std::cout << "CPU time " << (b - a) << " or TOTAL time in seconds " << time << " for " << str_test.str() << std::endl;
					std::cout << boost::format("CPU time %-5i total %-14i for %-10in") % (b - a) % time % str_test.str() << std::endl;
					

#else
					bool replaced = checkAndReplace(gp, gp_model, final_hyp, to_fuse, match, j, other, index_other, match_to_compare, k, size);	
#endif
					if(draw == true){
						std::cout << "THIS DRAW" << std::endl;
						final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "updated final", 1);
						_allhypothese_from_each_anchor[0].drawHypo(gp, gp_model, mat_in, mat_in, "final that should not change", 1);
						cv::waitKey(0);
					}
					//If it was replaced, we update the value of the for loop
					if(replaced == true){

						std::cout << "SOMETHING WAS REPLACED" << std::endl;
						
						//Restart the to fuse loop AND make sure we didn't empty to_fuse.
						//To do that : go back one on to_fuse and to max on final ->
						//Thus we finish the for loop and directly do ++k <=> same place but make sure we're not at size_fuse
						// start over final.
						j = final_hyp.size();
						//Because no removed in replace anymore
// 						--k;
// 						--size_fuse;
// 						if(match != other){
// 							//Minus one only if two match were removed.
// 							--size_final;
// 						}	
					}
					
				}


				
//				//Test the second part
//				else if(final_hyp[j].getSecond() == to_fuse[k].getSecond()){
//					std::cout << "SEEN2" << std::endl;
//					is_in_final = true;
//					//Both match to compare
//					Match match = final_hyp[j];
//					Match match_to_compare = to_fuse[k];
//
//					/* We found a common node in the input graph.
//					* We need to compare the two match to know if replacing it is an advantage or not
//					* But to compare it, maybe, the new match is matched in the model graph to a very good vertex.
//					* So we need to compare the new match to the match of the input AND the model in -->final<--.
//					* */
//					Match other = match;
//					int index_other = -1;
//					for(size_t f = 0 ; f < final_hyp.size() ; f++){
//						if(to_fuse[k].getFirst() == final_hyp[f].getFirst() && f != j ){
//							other = final_hyp[f];
//							index_other = f ;
//						}
//					}
//
//					if(draw == true){
//						Hypothese dm;
//						Hypothese dm3;
//						Hypothese dm2;
//						Hypothese dmweird;
//
//						dm.push_back(match);
//						dm3.push_back(other);
//						dm2.push_back(match_to_compare);
//// 						dmweird.push_back(final_hyp[9]);
//						if(match == other ){
//							std::cout << "SAME at " <<k << " " <<j << std::endl;
//						}
//						else{
//							std::cout << "Not the same at " <<k << " " <<j << std::endl;
//						}
//						std::cout << "that here " << std::endl;
//						dm.drawHypo(gp, gp_model, mat_in, mat_in, "match", 1);
//						dm3.drawHypo(gp, gp_model, mat_in, mat_in, "match2", 1);
//						dm2.drawHypo(gp, gp_model, mat_in, mat_in, "match to compare", 1);
//// 						dmweird.drawHypo(gp, gp_model, mat_in, mat_in, "weirdo", 1);
//
//						final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "final just before", 1);
//
//					}
//
//					bool replaced = checkAndReplace(gp, gp_model, final_hyp, to_fuse, match, j, other, index_other, match_to_compare, k, size);
//
//					if(draw == true){
//						final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "updated final", 1);
//						cv::waitKey(0);
//					}
//
//					if(replaced == true){
//
//						std::cout << "SOMETHING WAS REPLACED 2" << std::endl;
//						//Restart the to fuse loop
//						j = final_hyp.size();
//						//Because no removed in replace anymore
//// 						--k;
//// 						--size_fuse;
//// 						if(match != other){
//// 							//Minus one only if two match were removed.
//// 							--size_final;
//// 							std::cout << "humhum" << std::endl;
//// // 							assert(size_final == final_hyp.size());
//// 						}
//					}
//
//
//				}
				
//			}

					std::cout << "FIN : " << is_in_final << std::endl;
// assert(size_final == final_hyp.size());
		}


		inline void GraphMatcherAnchor::fuse_hypo(AASS::graphmatch::Hypothese& final_hyp, AASS::graphmatch::Hypothese& to_fuse, bool draw, cv::Size size, AASS::graphmatch::GraphPlace gp, AASS::graphmatch::GraphPlace gp_model){

			cv::Mat mat_in = cv::Mat::zeros(size, CV_8U);
			if(draw == true) {
				mat_in = cv::Mat::zeros(size, CV_8U);
			}


			if(draw){
				final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "final hypo at first", 1);
				to_fuse.drawHypo(gp, gp_model, mat_in, mat_in, "to_fuse at first", 1);
				cv::waitKey(0);
			}


#ifdef TIMED
			double time = 0 ;
#endif
//			std::cout << "NEW MATCH TO TESt is seen : " << is_in_final << std::endl;
//			Match match_final_to_compare;
//			Match match_fuse_to_compare;
//			Match other;

			//This step is needed to not study the match we add from fuse at the end of the deque
			size_t size_fuse = to_fuse.size();
			size_t size_final = final_hyp.size();

			std::vector<AASS::graphmatch::Match> to_add;

			for (auto match : to_fuse.getMatches()){
				std::cout << "New match to study" << std::endl;
#ifdef DEBUG
// 							std::cout << "Testing for double match in loop at " << k << " " << size_fuse << " " <<j << " " << size_final << std::endl;
// 							std::cout << "Testing to fuse : " << to_fuse.size() << std::endl;
				containsDoubleMatch(to_fuse);
				containsDoubleLink(to_fuse);
// 							std::cout << "Testing final : " << final.size() << std::endl;
				containsDoubleMatch(final_hyp);
				containsDoubleLink(final_hyp);
#endif
//				auto [match_final_to_compare, other_final_to_compare] = final_hyp.getLinkedMatches(match);

//				auto match = to_fuse[i];
				if(_use_lowest_value_for_matching && final_hyp.shouldReplaceBasedOnCost(match)){
					auto [match_final_to_compare, other_final_to_compare] = final_hyp.getLinkedMatches(match);
					int idx = 1;
					bool seen = final_hyp.isSeen(match_final_to_compare, idx);
					int idx_other = -1;
					bool seen_other = final_hyp.isSeen(other_final_to_compare, idx_other);

					to_add.push_back(match);

					final_hyp.remove(match_final_to_compare);
					if(match_final_to_compare != other_final_to_compare) {
						final_hyp.remove(other_final_to_compare);
						std::cout << "Removed an element" << std::endl;
					}

					if(draw == true){
						Hypothese dm;
						Hypothese dm3;
						Hypothese dm2;
						Hypothese dmweird;

						dm.push_back(match);
						dm3.push_back(other_final_to_compare);
						dm2.push_back(match_final_to_compare);
// 						dmweird.push_back(final_hyp[9]);
						std::cout << "that here " << std::endl;
						dm.drawHypo(gp, gp_model, mat_in, mat_in, "match", 1);
						dm3.drawHypo(gp, gp_model, mat_in, mat_in, "match other", 1);
						dm2.drawHypo(gp, gp_model, mat_in, mat_in, "match to compare", 1);
// 						dmweird.drawHypo(gp, gp_model, mat_in, mat_in, "weirdo", 1);

						cv::waitKey(0);

					}




//					bool replaced = replace(gp, gp_model, final_hyp, to_fuse, match_final_to_compare, idx, other_final_to_compare, idx_other, match, i);

					//Not needed because we iterate to_fuse
//					if(!replaced) {
//						++i;
//					}
				}



//				int aaaa;
//				std::cin>>aaaa;

			}

			std::cout << "Adding elements" << std::endl;

			//Adding all elements
			for(auto match : to_add){
				final_hyp.push_back(match);
			}

			if(draw){
				final_hyp.drawHypo(gp, gp_model, mat_in, mat_in, "final hypo after", 1);
				to_fuse.drawHypo(gp, gp_model, mat_in, mat_in, "to_fuse after", 1);
				cv::waitKey(0);
			}

			std::cout << "DONE" << std::endl;
		}
		
		
	}
}

#endif
