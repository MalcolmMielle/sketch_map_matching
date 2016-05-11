#ifndef VFLVISITOR_GRAPH
#define VFLVISITOR_GRAPH

#include <stdlib.h>     /* srand, rand */
#include "argraph.h"
#include "argedit.h"
#include "argloader.h"
#include <iostream>
#include <stdio.h>

namespace AASS{
	/**
	* @brief Store all corresponding node in vector<cv::Point2i>
	*/
	bool visitorVector(int n, node_id ni1[], node_id ni2[], void *usr_data){
		
		std::vector<std::pair < int, int > >* vect = (std::vector<std::pair < int, int > >*) usr_data;
		std::cout << std::endl << "NEW MATCHING " << std::endl;
		for(int i = 0 ; i < n ; ++i) {
			std::cout << "Node " << ni1[i] << " is paired with node " << ni2[i] << std::endl;
			vect->push_back(std::pair<int, int>(ni1[i], ni2[i]));
		}
		//Return true to only get the first matching
		return true;
	}
	
	
	/**
	 * @brief Extract all place of MATCHING -> Should be put in a wrapper
	 */
	
	void extractPlacefromVFL(){
		
	}
	
	
	
}




#endif