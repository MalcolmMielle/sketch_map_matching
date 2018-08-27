#ifndef UTILSGRAPH_MALCOLM_GRAPH
#define UTILSGRAPH_MALCOLM_GRAPH

#include "GraphPlace.hpp"
#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include "MatchingFunctions.hpp"
#include "Hypothese.hpp"

namespace AASS{
	
	//dirty debug function TEST
	inline std::string type2str(int type) {
	std::string r;

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
		case CV_8U:  r = "8U"; break;
		case CV_8S:  r = "8S"; break;
		case CV_16U: r = "16U"; break;
		case CV_16S: r = "16S"; break;
		case CV_32S: r = "32S"; break;
		case CV_32F: r = "32F"; break;
		case CV_64F: r = "64F"; break;
		default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');
	
	std::cout << "TYPE of MAt is " << r.c_str() << std::endl;

	return r;
	}
	
	
	namespace graphmatch{
		

		void createGraph(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp_copy, int nb_vertex, int nb_edges, std::pair< int, int >& size_image, std::deque< graphmatch::VertexPlace >& dplace, std::deque< graphmatch::VertexPlace >& dplace_copy);
		void copyGraph(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp2);
		
		//Does not return the good cost if the modified vertex is one of the new one :S
		int addRandomVertex(graphmatch::GraphPlace& gp, std::pair< int, int >& size_image, std::deque< graphmatch::VertexPlace >& dplace);
		int addRandomEdge(graphmatch::GraphPlace& gp, std::deque< graphmatch::VertexPlace >& dplace);
		
		bool containsDoubleMatch(const Hypothese& hyp);
		bool containsDoubleLink(const Hypothese& hyp);
		bool isRoom(const GraphPlace& gp, const VertexPlace& v);
		
		
		inline bool isRoom(const GraphPlace& gp, const VertexPlace& v)
		{
			int junc = gp.getNumEdges(v);
			if(gp[v].landmarks.size() == 1 && junc <= 2){
				return false;
			}
			return true;
		}
		
		inline void createGraph(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_copy, int nb_vertex, int nb_edges, std::pair< int, int >& size_image, std::deque< AASS::graphmatch::VertexPlace >& dplace, std::deque< AASS::graphmatch::VertexPlace >& dplace_copy)
		{
			
			
			int x_size = rand() % size_image.first + 1;
			int y_size = rand() % size_image.second + 1;
		
			graphmatch::VertexPlace v_previous;
			graphmatch::VertexPlace v_previous_copy;
//			std::vector <cv::Point> conte_test_editdistance;
			cv::Point2i mcnew_test_editdistance(x_size, y_size);
			cv::Moments m_test_editdistance;
			Place p;
			p.moment = m_test_editdistance;
//			p.contour = conte_test_editdistance;
			p.mass_center = mcnew_test_editdistance;
			gp.addVertex(v_previous, p);
			gp_copy.addVertex(v_previous_copy, p);
			dplace.push_back(v_previous);
			dplace_copy.push_back(v_previous_copy);
			
			//Create all the node in line
			for(int i = 1 ; i < nb_vertex ; i++){
				
				x_size = rand() % size_image.first + 1;
				y_size = rand() % size_image.second + 1;
			
				graphmatch::VertexPlace v;
				graphmatch::VertexPlace v_copy;
//				std::vector <cv::Point> conte_test_editdistance;
				cv::Point2i mcnew_test_editdistance(x_size, y_size);
				cv::Moments m_test_editdistance;
				p.mass_center = mcnew_test_editdistance;
				gp.addVertex(v, p);
				gp_copy.addVertex(v_copy, p);

				graphmatch::EdgePlace e;
				graphmatch::EdgePlace e_copy;
				graphmatch::Gateway gt_test_editdistance;
				gp.addEdge(e, v, v_previous, gt_test_editdistance);
				gp_copy.addEdge(e_copy, v_copy, v_previous_copy, gt_test_editdistance);
				
				int room_or_crossing = rand() % 1;
				if(room_or_crossing == 0){
					vodigrex::SimpleNode inter;
					bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex vert;
					std::pair <vodigrex::SimpleNode, bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex > pair(inter, vert);
					gp[v].landmarks.push_back(pair);
					gp_copy[v_copy].landmarks.push_back(pair);
				}
				else{
					vodigrex::SimpleNode inter;
					bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex vert;
					std::pair <vodigrex::SimpleNode, bettergraph::PseudoGraph<vodigrex::SimpleNode, vodigrex::SimpleEdge>::Vertex > pair(inter, vert);
					gp[v].landmarks.push_back(pair);
					gp_copy[v_copy].landmarks.push_back(pair);
				}
				
				v_previous = v;
				v_previous_copy = v_copy;
				dplace.push_back(v_previous);
				dplace_copy.push_back(v_previous_copy);
			}
			
			//Add some more edges
			for(int i = nb_vertex ; i < nb_edges ; i++){
				int v_tmp = rand() % dplace.size();
				int v_tmp_2 = rand() % dplace.size();
				while(v_tmp == v_tmp_2){
					v_tmp = rand() % dplace.size();
					v_tmp_2 = rand() % dplace.size();
				}
				
				graphmatch::VertexPlace vp = dplace[v_tmp];
				graphmatch::VertexPlace vp_2 = dplace[v_tmp_2];
				graphmatch::VertexPlace vp_copy = dplace_copy[v_tmp];
				graphmatch::VertexPlace vp_2_copy = dplace_copy[v_tmp_2];
				
				graphmatch::EdgePlace e;
				graphmatch::EdgePlace e_copy;
				graphmatch::Gateway gt_test_editdistance;
				gp.addEdge(e, vp, vp_2, gt_test_editdistance);
				gp_copy.addEdge(e_copy, vp_copy, vp_2_copy, gt_test_editdistance);
				
// 				int room_or_crossing = rand() % 1;
// 				if(room_or_crossing == 0){
// 					topologicalmap::Intersection_Graph inter;
// 					gp.addCrossing(vp, vp_2, inter);
// 					gp_copy.addCrossing(vp_copy, vp_2_copy, inter);
// 				}
// 				else{
// 					topologicalmap::Intersection_Graph inter;
// 					gp.addCrossing(vp, vp_2, inter);
// 					gp.addCrossing(vp, vp_2, inter);
// 					gp_copy.addCrossing(vp_copy, vp_2_copy, inter);
// 					gp_copy.addCrossing(vp_copy, vp_2_copy, inter);
// 				}
				
			}
			
			

		}
		
		inline void copyGraph(graphmatch::GraphPlace& gp, graphmatch::GraphPlace& gp2)
		{

		}

		/* Return the cost of the modification
		* 
		*/
		inline int addRandomVertex(graphmatch::GraphPlace& gp, std::pair<int, int>& size_image, std::deque< graphmatch::VertexPlace >& dplace)
		{
			
			
			
			int v_tmp = rand() % dplace.size();
			int x_size = rand() % size_image.first + 1;
			int y_size = rand() % size_image.second + 1;
			
			graphmatch::VertexPlace vp = dplace[v_tmp];
		
			bool isRoomt = isRoom(gp, vp);
			graphmatch::VertexPlace v_previous;
//			std::vector <cv::Point> conte_test_editdistance;
			cv::Point2i mcnew_test_editdistance(x_size, y_size);
			cv::Moments m_test_editdistance;
			Place p;
//			p.contour = conte_test_editdistance;
			p.moment = m_test_editdistance;
			p.mass_center = mcnew_test_editdistance;
			gp.addVertex(v_previous, p);
			dplace.push_back(v_previous);
			graphmatch::EdgePlace e;
			graphmatch::Gateway gt_test_editdistance;
			gp.addEdge(e, v_previous, vp, gt_test_editdistance);
			
			bool isRoom_now = isRoom(gp, vp);
			if(isRoom_now != isRoomt){
				return 3;
			}
			else{
				return 2;
			}

		}


		inline int addRandomEdge(graphmatch::GraphPlace& gp, std::deque< graphmatch::VertexPlace >& dplace)
		{

			int v_tmp = 0;
			int v_tmp_2 = 0;
			while(v_tmp == v_tmp_2){
				v_tmp = rand() % dplace.size();
				v_tmp_2 = rand() % dplace.size();
			}
			graphmatch::VertexPlace vp = dplace[v_tmp];
			graphmatch::VertexPlace vp_2 = dplace[v_tmp_2];
			
			bool isRoomt = isRoom(gp, vp);
			bool isRoom_2 = isRoom(gp, vp_2);
			
			graphmatch::EdgePlace e;
			graphmatch::Gateway gt_test_editdistance;
			bool exist = gp.addEdge(e, vp, vp_2, gt_test_editdistance);
			
			bool isRoom_now = isRoom(gp, vp);
			bool isRoom_2_now = isRoom(gp, vp_2);
	// 		std::cout << "Exist " << exist << std::endl;
			int oi = 0;
			if(isRoomt != isRoom_now){
				oi++;
			}
			if(isRoom_2 != isRoom_2_now){
				oi++;
			}
			if(exist == true){
	// 			std::cout << "TRUE EXIST" <<std::endl;
				return oi;
			}
			else{
				oi++;
				return oi;
			}

		}
		
		
		inline bool containsDoubleMatch(const Hypothese& hyp)
		{
			
			for(size_t i = 0 ; i < hyp.size() ; ++i){
				
				for(size_t j = i + 1 ; j < hyp.size() ; ++j){
					
					if( (hyp[i] == hyp[j]) == true){
						
						std::cout << "PROBLEM : the same match is present in the hypothese at " << i << "  " << j << std::endl;
						std::cout << hyp[i] << " and " << hyp[j] << std::endl;
						std::string str("PROBLEM : the same match is present in the hypothese");
						
						
						
						std::fstream fstr;
						fstr << str.c_str() << i << " " << j;
						
						
						throw std::runtime_error(str);
						return false;
						
					}
					
				}
			}
			
			return true;
		}
		
		inline bool containsDoubleLink(const Hypothese& hyp)
		{
			for(size_t i = 0 ; i < hyp.size() ; ++i){
				
				for(size_t j = i + 1 ; j < hyp.size() ; ++j){
					
					if(hyp[i].sameVertexThan(hyp[j]) ==  true){
						
						std::cout << "PROBLEM : the links conatins the same vertice at " << i << "  " << j << std::endl;
						std::string str("PROBLEM : the links conatins the same vertice at ");
						
						std::fstream fstr;
						fstr << str.c_str() << i << " " << j;
						
						
						throw std::runtime_error(str);
						return false;
						
					}
					
				}
			}
			
			return true;
		}


	}
		
	
}

#endif