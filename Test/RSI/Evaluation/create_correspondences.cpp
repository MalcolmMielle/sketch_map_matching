#include <iostream>
#include <time.h>
#include <cstdlib>
#include <fstream>
#include <ctime>
#include <sys/stat.h>
//#include <RSI/FuzzyOpening.hpp>
//#include <RSI/ZoneReducer.hpp>
//#include <RSI/ZoneExtractor.hpp>
#include <RSI/Uniqueness.hpp>

#include "RSI/GraphZoneRI.hpp"

#include "maoris/ZoneExtractor.hpp"
#include "maoris/FuzzyOpening.hpp"
//#include "maoris/Kmean.hpp"
#include "maoris/ZoneReducer.hpp"
#include "maoris/Segmentor.hpp"

#include "MatchMaps.hpp"

#include "Evaluation.hpp"


//struct MatchMaps{
//
//	MatchMaps(const std::string& name, const std::string& name2) : map1(name), map2(name2){};
//
//	std::string map1;
//	std::string map2;
//
//	std::vector<cv::Point2i> pt_map1;
//	std::vector<cv::Point2i> pt_map2;
//
//	void print(){
//		std::cout << "Point map1 : " << "\n";
//		for(auto point : pt_map1){
//			std::cout << point.x << " " << point.y << std::endl;
//		}
//
//		std::cout << "Point map2 : " << "\n";
//		for(auto point : pt_map2){
//			std::cout << point.x << " " << point.y << std::endl;
//		}
//	}
//
//	void export_all(std::ofstream& myfile){
//		myfile << "# map1" << "\n";
//		for(auto point : pt_map1){
//			myfile << point.x << " " << point.y << std::endl;
//		}
//
//		myfile << "# map2" << "\n";
//		for(auto point : pt_map2){
//			myfile << point.x << " " << point.y << std::endl;
//		}
//	}
//
//};
//
//struct MatchesBetweenMaps{
//
//	std::string map1;
//	std::string map2;
//	std::vector< MatchMaps > matches;
//
//	MatchesBetweenMaps(const std::string& name, const std::string& name2) : map1(name), map2(name2){};
//
//	void print(){
//		for(int i = 0 ; i < matches.size() ; ++i){
//			std::cout << "# " << i << "th match set\n";
//			matches[i].print();
//			std::cout << std::endl;
//		}
//	}
//
//	void export_all(const std::string& file_out){
//		std::string result_file = file_out;
//		std::ofstream myfile;
//		if(!exists_test3(result_file)){
//			myfile.open (result_file);
//			myfile << "# " << map1 << " " << map2 << "\n";
//		}
//		else{
//			myfile.open (result_file, std::ios::out | std::ios::app);
//			myfile << "\n# " << map1 << " " << map2 << "\n";
//		}
//
//		if (myfile.is_open())
//		{
//			for(int i = 0 ; i < matches.size() ; ++i){
//				myfile << i << " match set\n";
//				matches[i].export_all(myfile);
//				myfile << "\n";
//			}
//		}
//
//	}
//
//
//};

struct userdataCV{

	userdataCV(const std::string& name, const std::string& name2) : matchmaps( new AASS::graphmatch::evaluation::MatchMaps(name, name2) ) {};

	AASS::graphmatch::evaluation::MatchMaps* matchmaps;
//	std::string file;
	bool exit = false;
};

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{

	userdataCV* usrdatacv = (userdataCV*)userdata;

	if  ( event == cv::EVENT_LBUTTONDOWN )
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map1 << std::endl;
		usrdatacv->matchmaps->pt_map1.push_back(cv::Point2i(x, y));

	}
	else if  ( event == cv::EVENT_RBUTTONDOWN )
	{
		std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map1 << std::endl;
		usrdatacv->exit = true;
	}
	else if  ( event == cv::EVENT_MBUTTONDOWN )
	{
		std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map1 << std::endl;
	}
	else if ( event == cv::EVENT_MOUSEMOVE )
	{
		std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map1 << std::endl;

	}
}


void CallBackFunc2(int event, int x, int y, int flags, void* userdata)
{

	userdataCV* usrdatacv = (userdataCV*)userdata;

	if  ( event == cv::EVENT_LBUTTONDOWN )
	{
		std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map2 << std::endl;
		usrdatacv->matchmaps->pt_map2.push_back(cv::Point2i(x, y));

	}
	else if  ( event == cv::EVENT_RBUTTONDOWN )
	{
		std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map2 << std::endl;
		usrdatacv->exit = true;
	}
	else if  ( event == cv::EVENT_MBUTTONDOWN )
	{
		std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map2 << std::endl;
	}
	else if ( event == cv::EVENT_MOUSEMOVE )
	{
		std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << " in " << usrdatacv->matchmaps->map2 << std::endl;

	}
}







cv::Mat makeGraph(const std::string& file, AASS::RSI::GraphZoneRI& graph_slam){

	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
/** Segmenting the map**/
	AASS::maoris::Segmentor segmenteur;
	AASS::maoris::GraphZone graph_segmented;

	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	time = segmenteur.segmentImage(slam1, graph_segmented);
	cv::Mat segmented_map = segmenteur.getSegmentedMap();

//	cv::imshow("Segmented", segmented_map);
//	cv::waitKey(0);

	graph_slam = AASS::RSI::GraphZoneRI(graph_segmented);

	graph_slam.updatePCA();
	graph_slam.setPCAClassification();
	graph_slam.setSizesClassification();

	return segmented_map;
}

int main(int argc, char** argv){

	std::string file;
	std::string file2;
	std::string file_export;

	std::cout << "argc " << argc << std::endl;
	if(argc == 4){
		file = argv[1];
		file2 = argv[2];
		file_export = argv[3];
	}
	else{
		file  = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/Sketches/01.png";
		file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/Sketches/02.png";
		file_export = "export.dat";
	}

	//Segment maps
//	std::string file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/01.png";
	AASS::RSI::GraphZoneRI graph_slam_model;
	cv::Mat graph_slam_segmented = makeGraph(file, graph_slam_model);


//	std::string file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/02.png";
	AASS::RSI::GraphZoneRI graph_slam_model2;
	cv::Mat graph_slam_segmented2 = makeGraph(file2, graph_slam_model2);

	//Click corresponding region

	AASS::graphmatch::evaluation::MatchesBetweenMaps matches(file, file2);

	bool exit = false;
	while(exit == false) {
		userdataCV usrdataCV(file, file2);

		cv::namedWindow("Input", 1);
		cv::setMouseCallback("Input", CallBackFunc, &usrdataCV);
		imshow("Input", graph_slam_segmented);

		cv::namedWindow("Model", 1);
		cv::setMouseCallback("Model", CallBackFunc2, &usrdataCV);
		imshow("Model", graph_slam_segmented2);

		cv::waitKey(0);

		matches.matches.push_back(*usrdataCV.matchmaps);
		usrdataCV.matchmaps->print();
		exit = usrdataCV.exit;
	}

	std::cout << "All matches" << std::endl;
	matches.print();


	matches.export_all(file_export);


	AASS::graphmatch::evaluation::Evaluation ev;
	ev.read_file(file_export);

	//Save centers in file


}