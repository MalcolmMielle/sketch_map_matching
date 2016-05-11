#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
#include <iostream>
//TODO : test with empty graph


#include "GraphDB.hpp"
#include "GraphPlace.hpp"
#include "ConversionVFL.hpp"
#include "match.h"
#include "vf2_sub_state.h"
#include "argloader.h"
#include "GraphVFLLabeled.hpp"
#include <ctime>

//TODO Maybe add a node comparator 


bool my_visitor(int n, node_id ni1[], node_id ni2[], void *usr_data){
	
	std::cout << std::endl << "NEW MATCHING " << std::endl;
	for(int i = 0 ; i < n ; ++i) {
		std::cout << "Node " << ni1[i] << " is paired with node " << ni2[i] << std::endl;
	}
	return false;
}


std::vector<unsigned char> intToBytes(int paramInt)
{
     std::vector<unsigned char> arrayOfByte(4);
     for (int i = 0; i < 4; i++)
         arrayOfByte[i] = (paramInt >> (i * 8));
     return arrayOfByte;
}


BOOST_AUTO_TEST_CASE(trying)
{
	
	AASS::graphmatch::GraphPlace gp;
	
	//working on iso_m2D_m196.A00
	FILE *in = fopen("../Test/GraphDB/iso_m2D_m196.A00", "rb");
	
	char** matrix;
	int nodes = read_graph_unlabeled(in, &matrix);
// 	std::cout << " value " << matrix[0][3] << std::endl;
// 	std::cout << "NODES" << nodes << std::endl;
	gp.read(matrix, nodes);
	
	std::ofstream oo("graphhy.txt");
	
	gp.write(oo);
	
	
	
	/*** Conversion ***/
	
	AASS::graphmatch::GraphPlace gp_2;
	AASS::graphmatch::VertexPlace vp;
	AASS::graphmatch::VertexPlace vp2;
	AASS::graphmatch::VertexPlace vp3;
	AASS::graphmatch::Place place;
	place.mass_center.x = 10;
	place.mass_center.y = 50;
	gp_2.addVertex(vp, place);
	
	place.mass_center.x = 40;
	place.mass_center.y = 40;
	gp_2.addVertex(vp2, place);
	
	place.mass_center.x = 0;
	place.mass_center.y = 0;
	gp_2.addVertex(vp3, place);
	
	AASS::graphmatch::EdgePlace outedge;
	gp_2.addEdge(outedge, vp, vp2);
	gp_2.addEdge(outedge, vp, vp3);
	
	
	AASS::VFLGraph graph_vfl = AASS::graphPlace2VFL(gp_2);
	
	ofstream out("test_graphdb.txt");
	StreamARGLoader<AASS::graphmatch::Place, AASS::Empty>::write(out, graph_vfl);
	out.close();
	
	
	
	AASS::graphmatch::GraphPlace gp_3 = AASS::VFL2GraphPlace(graph_vfl);
	gp_3.print();
	
	
	VF2SubState s0_load(&graph_vfl, &graph_vfl);
	
	std::cout << std::endl << std::endl << "Second test " << std::endl;
	if(!match(&s0_load, my_visitor)){
		
		std::cout << "No matching found" << std::endl;
		
	}
	
	
	
	for(int y = 0 ; y < 3 ;y++){
		//malloc the 'x' dimension
		free(matrix[y]);
		//iterate over the 'x' dimension
		
	}
	free(matrix);
	
	/***
	 * TEST LOADING LABELED GRAPH
	 * 
	 * ***/
	std::cout << "LABELED GRAPH NOW" << std::endl;
	
	bettergraph::SimpleGraph<AASS::NodeSimple, AASS::Empty> gp_better;
	std::ifstream instr("../Test/GraphDB/si2_m2D_s36_00.grf");
	gp_better = AASS::getLabeledGraph(instr);
	
	std::ofstream outout("putain.txt");
	gp_better.write(outout);
	
	
	std::ifstream instr2("../Test/GraphDB/si2_m2D_s16_00.grf");
	AASS::graphmatch::GraphPlace graphplace = AASS::getLabeledGraph2Place(instr2);
	
	std::ifstream instr2_sub("../Test/GraphDB/si2_m2D_s16_00.sub.grf");
	AASS::graphmatch::GraphPlace graphplace_sub = AASS::getLabeledGraph2Place(instr2_sub);
	
	
	//Adding random second vertex position
	
// 	srand(time(NULL));
// 	std::vector<int> rando;
// 	
// 	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp_ite;
// 	for (vp_ite = boost::vertices( graphplace_sub.getGraph() ); vp_ite.first != vp_ite.second; ++vp_ite.first) {
// // 		std::cout << "Adding vertex" << std::endl;
// 		AASS::graphmatch::VertexPlace v = *vp_ite.first;
// 		int y = rand()%36;
// 		graphplace_sub[v].mass_center.y = y;
// 		rando.push_back(y);
// 
// 	}
// 	
// 	int count = 0 ;
// 	for (vp_ite = boost::vertices( graphplace.getGraph() ); vp_ite.first != vp_ite.second; ++vp_ite.first) {
// // 		std::cout << "Adding vertex" << std::endl;
// 		AASS::graphmatch::VertexPlace v = *vp_ite.first;
// 		if(count < rando.size()){
// 			graphplace[v].mass_center.y = rando[count];
// 			count++;
// 		}
// 		else{
// 			int y = rand()%36;
// 			graphplace[v].mass_center.y = y;
// 		}
// 	}
	
	//Separet the int in two 4 bit and then to 2 int
	
	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp_ite;
	for (vp_ite = boost::vertices( graphplace_sub.getGraph() ); vp_ite.first != vp_ite.second; ++vp_ite.first) {
		
		AASS::graphmatch::VertexPlace vertt = *vp_ite.first;
		int x = graphplace[vertt].mass_center.x;
		byte bytes[sizeof x];
		std::copy(static_cast<const byte*>(static_cast<const void*>(&x)),
				static_cast<const byte*>(static_cast<const void*>(&x)) + sizeof x,
          bytes);
		
		int i = 0 ;
		std::cout << "yop " << sizeof x << " for " << x << std::endl;
		std::cout << sizeof bytes << std::endl;
		
		while (i < sizeof(bytes))
		{
			printf("%02X",(int)bytes[i]);
			i++;
			std::cout << " i " << i << std::endl;
			
		}
		
		
		
// 		std::cout << "Adding vertex" << std::endl;
// 		AASS::graphmatch::VertexPlace v = *vp_ite.first;
// 		int y = rand()%36;
// 		graphplace_sub[v].mass_center.y = y;

	}
	
	
	
	/********************/
	
	cv::Mat paint = cv::Mat::zeros(800, 800, CV_8UC3);
	cv::Mat paint_sub = cv::Mat::zeros(800, 800, CV_8UC3);
	
	graphplace.scale(20);
	graphplace_sub.scale(20);
	
	graphplace.draw(paint);
	graphplace_sub.draw(paint_sub);
	
	cv::imshow("Graph", paint);
	cv::imshow("Graph_sub", paint_sub);
	cv::waitKey(0);
}
