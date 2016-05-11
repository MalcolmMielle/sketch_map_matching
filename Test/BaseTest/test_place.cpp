#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
//TODO : test with empty graph


#include "Place.hpp"
#include "Init.hpp"
#include "GraphPlace.hpp"

#include "PlaceExtractorList2Place.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "SketchMap.hpp"
#include "MapComparator/Util.hpp"

BOOST_AUTO_TEST_CASE(trying)


{

	
// 	AASS::graphmatch::AllKeypoints ak;
	
	AASS::vodigrex::ThinkerVoronoi t;
	AASS::graphmatch::PlaceExtractorList2Place p;
	
	AASS::SketchMap sk(t, p);
	
	cv::Mat mat_in = cv::imread("../Test/Simulation/FromVirtual/Gael/ObstacleMap.png");
	
	sk.setObstacleMat(mat_in);
	
	sk.think();
	sk.think();
	sk.think();
	
	AASS::graphmatch::GraphPlace gp;
	AASS::graphmatch::GraphPlace gp_cop;
	std::pair< int, int > size_image;
	size_image.first = 50;
	size_image.second = 100;
	std::deque< AASS::graphmatch::VertexPlace > dplace;
	std::deque< AASS::graphmatch::VertexPlace > dplace_copy;
	
	AASS::graphmatch::GraphPlace gp2;
	
	gp2 = gp;
	
	gp2.print();
	
	
	AASS::graphmatch::createGraph(gp, gp_cop, 5, 0, size_image, dplace, dplace_copy);
	
	std::ofstream out("outgraph.txt");
	gp.write(out);
	
	
	AASS::graphmatch::Place ppp;
	
	AASS::graphmatch::Place p2(ppp);
	
	AASS::graphmatch::GraphPlace gp_extr;
	std::ifstream in("outgraph.txt");
	gp_extr.read(in);
	
	gp_extr.print();
		
}