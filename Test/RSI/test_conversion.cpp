#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime> 
#include <RSI/FuzzyOpening.hpp>
#include <RSI/ZoneReducer.hpp>
#include <RSI/ZoneExtractor.hpp>

#include "RSIConversion.hpp"


void makeGraph(const std::string& file, AASS::RSI::GraphZone& graph_slam){
	
	
	cv::Mat slam = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
// 	
// 	cv::imshow("input", slam);
// 	cv::waitKey(0);
	
	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY);
	cv::threshold(slam, slam, 20, 255, cv::THRESH_BINARY_INV);
	
	std::cout << "/************ FUZZY OPENING*************/ \n";
	AASS::RSI::FuzzyOpening fuzzy_slam;
	fuzzy_slam.fast(false);
	
	cv::Mat out_slam;
// 	cv::imshow("SLAM", slam);
// 	cv::waitKey(0);
	fuzzy_slam.fuzzyOpening(slam, out_slam, 500);
	std::cout << "Done opening " << std::endl;
	out_slam.convertTo(out_slam, CV_8U);
	
// 	std::cout << out << std::endl;
	
	std::cout << "/************ REDUCING THE SPACE OF VALUES *****************/\n";
	cv::Mat out_tmp_slam;
	AASS::RSI::reduceZone(out_slam, out_tmp_slam);
// 	cv::imshow("REDUCED", out_tmp_slam);
// 	cv::waitKey(0);
	
	AASS::RSI::ZoneExtractor zone_maker;
	std::cout << "WHATERSHED SLAM" << std::endl;
	zone_maker.extract(out_tmp_slam);
	
	std::cout << "Got the ZONES" << std::endl;

	// 	std::cout << "Getting the graph" << std::endl;
	
	std::cout << "/*********** MAKING AND TRIMMING THE GRAPH ***************/\n";
	graph_slam = zone_maker.getGraph();
	graph_slam.removeVertexValue(0);

	std::cout << "Number of nodes" << graph_slam.getNumVertices() << std::endl;
	
	//Watershed Algorithm
	graph_slam.watershed(0.25);
	
	int size_to_remove = 100;
	graph_slam.removeVertexUnderSize(size_to_remove, true);
	graph_slam.removeLonelyVertices();
	if(graph_slam.lonelyVertices())
		throw std::runtime_error("Fuck you lonelyness");	
	
	cv::Mat graphmat2 = cv::Mat::zeros(out_tmp_slam.size(), CV_8U);
	graph_slam.draw(graphmat2);
	cv::imshow("end", graphmat2);
}


BOOST_AUTO_TEST_CASE(trying)
{
	AASS::RSI::Zone zone;
	AASS::RSI::Zone zone2;
	
	AASS::graphmatch::ZoneKeypoint* zk = new AASS::graphmatch::ZoneKeypoint();
	zk->setZone(zone);
	zk->name = "1";
	AASS::graphmatch::Keypoint* re = static_cast<AASS::graphmatch::ZoneKeypoint*>(zk);
	
	AASS::graphmatch::ZoneKeypoint* zk2 = new AASS::graphmatch::ZoneKeypoint();
	zk2->setZone(zone2);
	zk2->name = "2";
	AASS::graphmatch::Keypoint* re2 = static_cast<AASS::graphmatch::ZoneKeypoint*>(zk2);
	
	
// 	bool res = re->compareKeypoints(re2);
// 	std::cout << "Same ? " << res << std::endl;
	
	/// TEST CONVERSION
	
// 	int argc = boost::unit_test::framework::master_test_suite().argc;
// 	char** argv = boost::unit_test::framework::master_test_suite().argv;
	std::string file;
	file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/00.png";
	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);

	AASS::RSI::GraphZone graph_slam;
	makeGraph(file, graph_slam);
		
	
	/********** PCA of all zones in Graph and removing the ripples **********/
	
// 	graph_slam.updatePCA();
// 	graph_slam.removeRiplesv2();
// 	graph_slam.updateContours();
	graph_slam.update();
	
	std::cout << "Size of graph" << graph_slam.getNumVertices() << std::endl;

	
// 	std::cout << "Size of graph2" << graph_slam2.getNumVertices() << std::endl;
	
	/********** Drawing the graphs *******************************************/
	
	cv::Mat graphmat = cv::Mat::zeros(slam1.size(), CV_8U);
	graph_slam.draw(graphmat);
	
	
	cv::imshow("graph1", graphmat);
	cv::waitKey(0);
	
	AASS::graphmatch::GraphPlace gp;
	
	AASS::graphmatch::graphZoneToGraphPlace(graph_slam, gp);
	
	cv::Mat m = cv::Mat::zeros(slam1.size(), CV_8U);
	gp.draw(m);
	cv::imshow("graph1 m", m);
	cv::waitKey(0);
	
}