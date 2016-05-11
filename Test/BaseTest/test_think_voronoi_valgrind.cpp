#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 

#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"

void timed(AASS::SketchMap& m){
	std::clock_t a = std::clock();
    m.think();
    std::clock_t b = std::clock();
    std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< std::endl;
}



BOOST_AUTO_TEST_CASE(trying)
{
// 	AASS::Thinker_Voronoi  t;
// 	std::cout << "Going in the map"<<std::endl;
// 	
// 	AASS::SketchMap m(500, 500, t);
// 	
// 	m.importFile2System();
// 	m.setDownSample(1);
// 	
// 	//Launch every test
// 	t.modePartialNormalized();
// 	std::cout << "PartialNormalized" << std::endl;
// 	timed(m);
// 	
// 	t.modeSobelLabel();
// 	std::cout << "Sobel on label" << std::endl;
// 	timed(m);
// 	
// 	t.modeLaplaceLabel();
// 	std::cout << "Laplace on Label" << std::endl;
// 	timed(m);
// 	
// 	t.modeCannyVoro();
// 	std::cout << "Sobel on distance" << std::endl;
// 	timed(m);
// 	
// 	t.modeLaplaceVoro();
// 	std::cout << "Laplace on Distance" << std::endl;
// 	timed(m);
// 	
// 	t.modeLocalMaxima();
// 	std::cout << "LocalMaxima" << std::endl;
// 	timed(m);
// 	
// 	t.modeLocalMaximaBest();
// 	std::cout << "LocalMaixmaBest" << std::endl;
// 	timed(m);
// 	
// // 	t.modeDelaunay();
// // 	std::cout << "Delaunay" << std::endl;
// // 	timed(m);	
// 	
// 	t.modeLocalMaximaBest();
// 	std::cout << "LocalMaixmaCombo" << std::endl;
// 	timed(m);
	
	
	
	std::cout << "SECOND TEST WITH GIMP IMAGE ******************************* /" << std::endl;
	int l = 255;
	AASS::vodigrex::ThinkerVoronoi tv;
	tv.setDownSample(1);
	tv.modeLaplaceVoro();
	cv::Mat mm = cv::imread("../Test/BaseTest/ObstacleMapBug.png");
	cv::Mat copy;
	tv.setDownSample(1);

	while(l != 0){
		copy = mm.clone();
		tv.think(copy);
		
		cv::imshow("image", mm);
		cv::imshow("result", tv.getResult());
		cv::waitKey(50);
		std::cin >> l;
		
		tv.setLevel(l);
		tv.reset();
	}
	
}