#include "GraphMatcherAnchor.hpp"


//
//bool AASS::graphmatch::GraphMatcherAnchor::bestMatch(const AASS::graphmatch::Match& match_original, const AASS::graphmatch::Match& match_maybe, const AASS::graphmatch::Match& match_to_compare, size_t& diff) const
//{
//// 	std::cout << "cost match " << match_original.getCost() << " maybe " << match_maybe.getCost() << " to compare " << match_to_compare.getCost() << std::endl;
//
//	//Extract actual difference to know if it's zero and they are the same
//	bool better = match_to_compare.isBetterThan(match_original, diff);
//	//If there is a second match to compare. That first comparison is useless but it make the code clearer in my opinion.
//	if(match_maybe != match_original){
//		size_t diff2 = 0;
//		if(better == true && match_to_compare.isBetterThan(match_maybe, diff2)){
//			diff += diff2;
//			better = true;
//		}
//		else{
//			better = false;
//		}
//	}
//	return better;
//}

bool AASS::graphmatch::GraphMatcherAnchor::checkAndReplace(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final, AASS::graphmatch::Hypothese& to_fuse, const AASS::graphmatch::Match& match_original, int index_match_original, const AASS::graphmatch::Match& match_maybe, int index_match_maybe, const AASS::graphmatch::Match& match_to_compare, int index_match_to_compare, cv::Size size) {

	std::cout << "Les couts " << match_original.getCost() << " " << match_to_compare.getCost() << "Les other ? "
	          << index_match_maybe << " " << match_maybe.getCost() << std::endl;

	bool better = false;
	if (_use_lowest_value_for_matching) {
		size_t diff = 0;
		better = match_original.bestMatch(match_maybe, match_to_compare, diff);
// 	std::cout << "COST : " << better << std::endl; 
		//The new match got more confidence than both others.
		std::cout << "DIFFERENCE " << diff << std::endl;
	} else if (_use_zone_of_equal_value_for_matching) {
		//It's pretty equal here... So I should use the local detection to know if I should replace. This should become the basic function later on. Not just for the 0 case.
//		size_t diff = 0;
		//Should detect the one with the largest zone with the same or less value of modification in their respective graphs.
// 		std::cout << "STUF TO IMPLEMENT IN REPLACE IN GRAPHMATCHER ANCHOR" << std::endl;
// 		throw std::runtime_error("GO FIX THE PROGRAM : STUFF TO TEST IN REPLACE IN GRAPHMATCHER ANCHOR");


		/* 1 -> Calculate zone with same or less for both vertex
		 * 2 -> Compare them
		 * 3 -> If they are the same FUCK MY LIFE 
		 */

// 		std::cout << "Compare the final because they are equal" << std::endl;
//TODO : Replace draw by preprocessor thing to be able to remove those hypothese
		Hypothese zone;
		Hypothese zone1;
		Hypothese zone2;
		Hypothese zone3;

		/*********************************************************************/
		//TODO : MAKE SURE THAT THE INDEX SELECTED IS THE GOOD ONE 
		/*********************************************************************/


		std::cout << "_from_which_hypo " << _from_which_hypo.size() << " index_match_original " << index_match_original
		          << std::endl;
		int which = _from_which_hypo[index_match_original];
		std::cout << "With " << which << std::endl;
// 		_allhypothese_from_each_anchor[0].getMatches()

		std::cout << "IN which " << which << " and fuse " << index_match_original << " which should be less than "
		          << _allhypothese_from_each_anchor[which].size() << std::endl;
		int size_final_1 = -1;
		int size_final_2 = -1;
		int size_to_fuse_1 = -1;
		int size_to_fuse_2 = -1;
#ifdef DEBUG
		try {
#endif
			size_final_1 = _allhypothese_from_each_anchor[which].getSizeSimilarZone(gp, match_original.getFirst(),
			                                                                        zone);
			size_final_2 = _allhypothese_from_each_anchor[which].getSizeSimilarZone(gp_model,
			                                                                        match_original.getSecond(), zone1);
			// 		std::cout << "Compare the fuse" << std::endl;
#ifdef DEBUG
		}
		catch (std::exception &e) {

			std::cout << "DRAWINF ZONE size of first and update" << _allhypothese_from_each_anchor[which].size() << " "
			          << final.size() << " at " << index_match_original << std::endl;
			cv::Mat mat_in = cv::Mat::zeros(size, CV_8U);;
			Hypothese dm;
			dm.push_back(match_original);
			dm.drawHypo(gp, gp_model, mat_in, mat_in, "DM", 1);
			_allhypothese_from_each_anchor[which].drawHypo(gp, gp_model, mat_in, mat_in, "Hyp", 1);
			final.drawHypo(gp, gp_model, mat_in, mat_in, "Hyp real", 1);
			cv::waitKey(0);

		}
		try {
#endif

			size_to_fuse_1 = _allhypothese_from_each_anchor[_studied_index_to_fuse].getSizeSimilarZone(gp,
			                                                                                           match_to_compare.getFirst(),
			                                                                                           zone2);
			size_to_fuse_2 = _allhypothese_from_each_anchor[_studied_index_to_fuse].getSizeSimilarZone(gp_model,
			                                                                                           match_to_compare.getSecond(),
			                                                                                           zone3);
#ifdef DEBUG
		}
		catch (std::exception &e) {
			std::cout << "DRAWINF ZONE to fuse size of first and update1"
			          << _allhypothese_from_each_anchor[_studied_index_to_fuse].size() << " " << to_fuse.size()
			          << " at " << index_match_to_compare << std::endl;
			cv::Mat mat_in = cv::Mat::zeros(size, CV_8U);;
			Hypothese dm;
			dm.push_back(match_to_compare);
			dm.drawHypo(gp, gp_model, mat_in, mat_in, "DM", 2);
			_allhypothese_from_each_anchor[_studied_index_to_fuse].drawHypo(gp, gp_model, mat_in, mat_in, "Hyp", 2);
			to_fuse.drawHypo(gp, gp_model, mat_in, mat_in, "Hyp real", 2);
			cv::waitKey(0);
		}
#endif


		if (size_to_fuse_1 + size_to_fuse_2 > size_final_1 + size_final_2) {
			std::cout << "It's better since " << size_to_fuse_1 + size_to_fuse_2 << " > " << size_final_1 + size_final_2
			          << std::endl;
//			if (diff == 0) {
				better = true;
//			}
		} else {
			std::cout << "It's no better since " << size_to_fuse_1 + size_to_fuse_2 << " < "
			          << size_final_1 + size_final_2 << std::endl;
		}

		if (_draw == true) {
			std::cout << "DRAWING ZONE" << std::endl;
			cv::Mat mat_in = cv::Mat::zeros(size, CV_8U);;
			zone.drawHypo(gp, gp_model, mat_in, mat_in, "zone", 1);
			zone2.drawHypo(gp, gp_model, mat_in, mat_in, "zone2", 1);
			cv::waitKey(0);
		} else {
			std::cout << "Not drawing " << std::endl;
		}
	}

	// Replacement step
	if(better == true){
// 		std::cout << "FUSE at " << index_match_original << " and to fuse " << index_match_to_compare << std::endl;
// 		std::cout << "removing " << final[index_match_original] << " adding " << to_fuse[index_match_to_compare] << " SHOULD EB SEAME" << match_to_compare << std::endl;
		replace(gp, gp_model, final, to_fuse, match_original, index_match_original, match_maybe, index_match_maybe, match_to_compare, index_match_to_compare);
		
	}
	
	return better;

}



bool AASS::graphmatch::GraphMatcherAnchor::replace(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model, AASS::graphmatch::Hypothese& final, AASS::graphmatch::Hypothese& to_fuse, const AASS::graphmatch::Match& match_original, int index_match_original, const AASS::graphmatch::Match& match_maybe, int index_match_maybe, const AASS::graphmatch::Match& match_to_compare, int index_match_to_compare)
{

	//Remove old matches from final
	final.erase (final.begin() + index_match_original);
	_from_which_hypo.erase(_from_which_hypo.begin() + index_match_original);
// 		to_fuse.erase (to_fuse.begin() + index_match_to_compare);
	if(index_match_maybe != -1){
// 			erase depend on posisiton of match_maybe compare to original
		if(index_match_maybe > index_match_original){
			//-1 because we already erase one vertex before before
			final.erase (final.begin() + index_match_maybe - 1);
			_from_which_hypo.erase (_from_which_hypo.begin() + index_match_maybe - 1);
		}
		else{
			final.erase (final.begin() + index_match_maybe);
			_from_which_hypo.erase (_from_which_hypo.begin() + index_match_maybe);
		}
	}
	else{
// 			std::cout << "Same so no double erase" << std::endl;
	}
	final.push_back(match_to_compare);
	//push back the number of the associated hypothese.
	_from_which_hypo.push_back(_studied_index_to_fuse);
	
	return true;
}