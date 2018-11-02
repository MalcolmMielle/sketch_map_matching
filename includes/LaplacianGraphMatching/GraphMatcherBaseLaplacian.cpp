#include "GraphMatcherBaseLaplacian.hpp"



void AASS::graphmatch::GraphMatcherBaseLaplacian::sort(
		std::deque<	graphmatch::HypotheseLaplacian>& hypothesis)
{

	//classify them in a clock wise manner
	std::deque<
			graphmatch::HypotheseLaplacian
	>::iterator hypothesis_final_ite;

	std::deque<
			graphmatch::HypotheseLaplacian
	>::iterator hypothesis_final_ite_2;

	graphmatch::HypotheseLaplacian copy;
// 	std::cout << "SIZE " << hypothesis.size() << std::endl;

	for(hypothesis_final_ite =  hypothesis.begin()+1 ; hypothesis_final_ite !=  hypothesis.end() ; hypothesis_final_ite++){

		hypothesis_final_ite_2 = hypothesis_final_ite ;
		copy = *hypothesis_final_ite;
// 			std::cout << "tsart" << std::endl;
// 			std::cout << "something" <<  ( * (hypothesis_final_ite_2 - 1) ).getDist() << " > " << copy.getDist()  << std::endl;

		while( hypothesis_final_ite_2 !=  hypothesis.begin() && (*(hypothesis_final_ite_2 - 1)).getDist() > copy.getDist()){

// 				std::cout << "MOVE" << std::endl;
			*( hypothesis_final_ite_2 ) = *( hypothesis_final_ite_2-1 );
			hypothesis_final_ite_2 = hypothesis_final_ite_2 - 1;


		}
		*(hypothesis_final_ite_2) = copy;
	}


}


void AASS::graphmatch::GraphMatcherBaseLaplacian::sort_by_size(
		std::deque<	graphmatch::HypotheseLaplacian>& hypothesis)
{

	//classify them in a clock wise manner
	std::deque<
			graphmatch::HypotheseLaplacian
	>::iterator hypothesis_final_ite;

	std::deque<
			graphmatch::HypotheseLaplacian
	>::iterator hypothesis_final_ite_2;

	graphmatch::HypotheseLaplacian copy;

	for(hypothesis_final_ite =  hypothesis.begin()+1 ; hypothesis_final_ite !=  hypothesis.end() ; hypothesis_final_ite++){

		hypothesis_final_ite_2 = hypothesis_final_ite ;
		copy = *hypothesis_final_ite;
// 			std::cout << "tsart" << std::endl;
// 			std::cout << "something" <<  ( * (hypothesis_final_ite_2 - 1) ).getDist() << " < " << (*hypothesis_final_ite_2).getDist()  << std::endl;
		while( hypothesis_final_ite_2 !=  hypothesis.begin() && (*(hypothesis_final_ite_2 - 1)).size() < copy.size()){

// 				std::cout << "MOVE" << std::endl;
			*( hypothesis_final_ite_2 ) = *( hypothesis_final_ite_2-1 );
			hypothesis_final_ite_2 = hypothesis_final_ite_2 - 1;


		}
		*(hypothesis_final_ite_2) = copy;
	}


}








void AASS::graphmatch::GraphMatcherBaseLaplacian::reset()
{

	_hypothesis_final.clear();

}

// void graphmatch::GraphMatcherBase::draw()
// {
// 	cv::Mat mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
//
// 	cv::Mat draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	cv::Mat draw_graph_reduced_2 = cv::Mat::zeros(mat_in.size(), CV_8UC3);
// 	_graph_place.drawSpecial(draw_graph_reduced);
// 	_graph_place_model.drawSpecial(draw_graph_reduced_2);
// 	cv::imshow("GraphMatcherBaseInput", draw_graph_reduced);
// 	cv::imshow("GraphMatcherBaseModel", draw_graph_reduced_2);
// 	cv::waitKey(0);
//
// }
