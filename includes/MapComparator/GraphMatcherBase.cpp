#include "GraphMatcherBase.hpp"

// void AASS::graphmatch::GraphMatcherBase::inputMaps(const SketchMap& m, const SketchMap& mm)
// {
// 
// 	_map_one = m;
// 	_map_model = mm;
// 	_graph_list = m.getGraphLines();
// 	_graph_list_model = mm.getGraphLines();
// 	_graph_place = m.getGraphPlace();
// 	_graph_place_model = mm.getGraphPlace();
// 	
// }
// 
// void AASS::graphmatch::GraphMatcherBase::inputNewMap(const SketchMap& m)
// {
// 	_map_one = m;
// 	_graph_list = m.getGraphLines();
// 	_graph_place = m.getGraphPlace();
// }
// 
// 
// void AASS::graphmatch::GraphMatcherBase::inputNewModel(const SketchMap& m)
// {
// 	_map_model = m;
// 	_graph_list_model = m.getGraphLines();
// 	_graph_place_model = m.getGraphPlace();
// }
// 
// 
// 
// 
// void AASS::graphmatch::GraphMatcherBase::init(const graphmatch::GraphPlace& gp, const graphmatch::GraphPlace& gp2)
// {
// 	
// 	_graph_place = gp;
// 	_graph_place_model = gp2;
// 	_num_vertex = gp.getNumVertices();
// 	_num_vertex_model = gp2.getNumVertices();
// }


void AASS::graphmatch::GraphMatcherBase::sort(
	std::deque<	graphmatch::Hypothese>& hypothesis)
{

	//classify them in a clock wise manner
	std::deque<	
		graphmatch::Hypothese
	>::iterator hypothesis_final_ite;
	
	std::deque<	
		graphmatch::Hypothese
	>::iterator hypothesis_final_ite_2;
	
	graphmatch::Hypothese copy;
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


void AASS::graphmatch::GraphMatcherBase::sort_by_size(
	std::deque<	graphmatch::Hypothese>& hypothesis)
{

	//classify them in a clock wise manner
	std::deque<	
		graphmatch::Hypothese
	>::iterator hypothesis_final_ite;
	
	std::deque<	
		graphmatch::Hypothese
	>::iterator hypothesis_final_ite_2;
	
	graphmatch::Hypothese copy;
	
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


void AASS::graphmatch::GraphMatcherBase::pairWiseMatch(
	const graphmatch::GraphPlace& gp, 
	const graphmatch::GraphPlace& gp2, 
	std::deque< 
		graphmatch::Match >& places_pair)
{
	
	
	std::pair<VertexIteratorPlace, VertexIteratorPlace> vp;
	for (vp = boost::vertices(gp.getGraph()); vp.first != vp.second; vp.first++) {
	
		VertexPlace v = *vp.first;
// 		bool is_room = gp.isRoom(v);
		std::string is_type = gp[v].getID();
		
		std::pair<VertexIteratorPlace, VertexIteratorPlace> vp2;
		for (vp2 = boost::vertices(gp2.getGraph()); vp2.first != vp2.second; vp2.first++) {
		
			VertexPlace v2 = *vp2.first;
			
// 			std::deque< std::pair < topologicalmap::EdgePlace, VertexPlace > > all_linked_edge;
// 			gp.getAllEdgeLinked(v, all_linked_edge);
// 			
// 			std::deque< std::pair < topologicalmap::EdgePlace, VertexPlace > > all_linked_edge2;
// 			gp.getAllEdgeLinked(v2, all_linked_edge2);
			
// 			bool is_room_2 = gp2.isRoom(v2);
			std::string is_type_model = gp2[v2].getID();
			
			//ATTENTION
			//Same number of linked room + same type of room. Maybe it's too big an assumption that they have the same number of links
// 			if(/*all_linked_edge.size() == all_linked_edge2.size() &&*/ is_room_2 == is_room){
			if(is_type == is_type_model){
				//Adding to the list of the vertex are of the same type
				places_pair.push_back(graphmatch::Match(v, v2) );
			}
			
		}
				
	}
}





void AASS::graphmatch::GraphMatcherBase::reset()
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
