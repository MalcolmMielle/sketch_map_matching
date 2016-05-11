#include "StorageFilter.hpp"

void AASS::graphmatch::StorageFilter::reduce(const std::deque< graphmatch::VertexPlace >& h, graphmatch::GraphPlace& gp)
{

	graphmatch::VertexPlace vp;
	//First get the first vertex of the hypo
	vp = h[0];
	gp.print(vp);
	for(size_t i = 1; i < h.size() ; i++){
		
		//Get all vertex linked to the vertex in the hypo
		std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > > pair;
		gp.getAllEdgeLinked(h[i], pair);
		
		for(size_t j = 0 ; j < pair.size() ; j ++){
			graphmatch::Gateway g;
			graphmatch::EdgePlace out;
			
			//Like all those vertex to the first vertex
			//if is just there to avoid linking to itself.
			if(vp != pair[j].second){
				bool exist = gp.addEdge(out, vp, pair[j].second, g);
				if(exist == false){
					_added_edges.push_back(out);
				}
				else{
// 					std::cout << "Edge deja la" << std::endl;
				}
				
			}
			else{
// 				std::cout << "SAMMMME" << std::endl;
			}
		}
		
		graphmatch::VertexPlace vv = h[i];
		addToRemove(vv);
	}
}


void AASS::graphmatch::StorageFilter::removeAllFalseLabel(const graphmatch::Filtered_place& fgraph, graphmatch::GraphPlace& gp)
{
	//vertices access all the vertix
	std::pair<graphmatch::VertexIteratorPlaceFiltered, graphmatch::VertexIteratorPlaceFiltered> vp;
	//vertices access all the vertix
	for (vp = boost::vertices(fgraph); vp.first != vp.second; ) {
// 			std::cout << "studying vertex " << std::endl;
		
		graphmatch::VertexPlace v = *vp.first;
		
		if(fgraph[v].label == false){
			
// 				std::cout << "removing vertex " << std::endl;
			std::deque< std::pair< graphmatch::EdgePlaceFiltered, graphmatch::VertexPlaceFiltered > > pair;
			gp.getAllEdgeLinked(v, pair);
			
			for(size_t i = 0 ; i < pair.size() ; i++){
// 					std::cout << "creating edges " << std::endl;
				for(size_t j = i+1 ; j < pair.size() ; j++){
					graphmatch::Gateway gg;
					graphmatch::EdgePlace ep;
					bool exist = gp.addEdge(ep, pair[i].second, pair[j].second, gg);
					if(exist == false){
// 							std::cout << "PushedEdge"<<std::endl;
						_added_edges.push_back(ep);
					}
					else{
// 							std::cout << "Edge deja la" << std::endl;
					}
				}
			}
			
// 			ATTENTION : It's important to do ++vp BEFORE adding it to addToRemove
			++vp.first;
			addToRemove(v);
// 				std::cout << "DONE " << std::endl;
		}
		else{
			++vp.first;
		}
		
	}
	

}


void AASS::graphmatch::StorageFilter::drawSpecialFiltered(const graphmatch::Filtered_place& fp, cv::Mat& m) const
{
	
	cv::Scalar color;
	if(m.channels() == 1){
		color = 255;
	}
	else if(m.channels() == 3){
		color[1] = 255;
		color[2] = 255;
		color[3] = 255;
	}
	
	cv::Scalar color_corner;
	if(m.channels() == 1){
		color_corner = 100;
	}
	else if(m.channels() == 3){
		color_corner[1] = 0;
		color_corner[2] = 255;
		color_corner[3] = 255;
	}
	
	//first is beginning, second is "past the end"
	std::pair<graphmatch::VertexIteratorPlaceFiltered, graphmatch::VertexIteratorPlaceFiltered> vp;
	//vertices access all the vertix
	for (vp = boost::vertices(fp); vp.first != vp.second; ++vp.first) {
		
		graphmatch::VertexPlaceFiltered v = *vp.first;
		
		drawSpecialFiltered(fp, m, v, color_corner);
	
		
		graphmatch::EdgeIteratorPlaceFiltered out_i, out_end;
		graphmatch::EdgePlaceFiltered e;
		
		for (boost::tie(out_i, out_end) = boost::out_edges(v, fp); 
			out_i != out_end; ++out_i) {
			e = *out_i;
			graphmatch::VertexPlaceFiltered src = boost::source(e, fp), targ = boost::target(e, fp);
			cv::line(m, fp[src].mass_center, fp[targ].mass_center, color);

		}
		
	}
}


void AASS::graphmatch::StorageFilter::drawSpecialFiltered(const graphmatch::Filtered_place& fp, cv::Mat& m, const graphmatch::VertexPlaceFiltered& v, const cv::Scalar& color) const
{
	cv::circle(m, fp[v].mass_center, 10, color, -1);
}


void AASS::graphmatch::StorageFilter::getAllEdgeLinked(const graphmatch::VertexPlace& v, const graphmatch::Filtered_place& fp, std::deque< std::pair< graphmatch::EdgePlace, graphmatch::VertexPlace > >& all_edge) const
{
// 		std::cout << "GRAPH PLACE EDGE LINKED" << std::endl;
	graphmatch::EdgeIteratorPlaceFiltered out_i, out_end;
	graphmatch::EdgePlace e;
	
	for (boost::tie(out_i, out_end) = boost::out_edges(v, fp); 
		out_i != out_end; ++out_i) {
		e = *out_i;
		graphmatch::VertexPlace targ = boost::target(e, fp);
		all_edge.push_back(std::pair<graphmatch::EdgePlace, graphmatch::VertexPlace> (e, targ) );
	}
}


void AASS::graphmatch::StorageFilter::drawAll(const std::string& str, const AASS::graphmatch::Filtered_place& fp, const AASS::graphmatch::GraphPlace& gp) const
{

	std::string name = str + "normal";

	cv::Mat mat_in = cv::imread("../Test/TEST_COMPARISON/TEST1/map/map.png");
	cv::Mat draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	gp.drawSpecial(draw_graph_reduced);
	cv::imshow(name, draw_graph_reduced);
	
	
	std::string name2 = str + "filtered";
	
	draw_graph_reduced = cv::Mat::zeros(mat_in.size(), CV_8UC3);
	drawSpecialFiltered(fp, draw_graph_reduced);
	cv::imshow(name2, draw_graph_reduced);

	
}