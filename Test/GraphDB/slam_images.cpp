#include "GraphDB.hpp"
#include "GraphPlace.hpp"
#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>
#include <boost/filesystem/path.hpp>
#include <cstdlib>
#include "Thinker_EVG.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "MultipleLineFollower.hpp"
#include "PlaceExtractorList2Place.hpp"


void ImgProc(cv::Mat& testt){
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
	int number = 20;
	for(int i = 0 ; i < 2 ; ++i){
		cv::erode(testt, testt, kernel);
	}
	for(int i = 0 ; i < number ; ++i){
		cv::dilate(testt, testt, kernel);
	}
	for(int i = 0 ; i < number ; ++i){
		cv::erode(testt, testt, kernel);
	}
}


AASS::graphmatch::GraphPlace getGraphPlace(AASS::topologicalmap::GraphList& gl_real, cv::Mat& test){

	AASS::graphmatch::PlaceExtractorList2Place extract;
	extract.inputMapIn(test);
	extract.inputGraph(gl_real);
// 	std::cout << "Extracting" << std::endl;
	extract.extract();
	return extract.getGraph();	
}


AASS::topologicalmap::GraphList getGraphListEVG(cv::Mat& testin){
	cv::Mat test;
	testin.copyTo(test);
	
// 	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
// 	int number = 20;
// 	for(int i = 0 ; i < number ; ++i){
// 		cv::dilate(test, test, kernel);
// 	}
// 	for(int i = 0 ; i < number ; ++i){
// 		cv::erode(test, test, kernel);
// 	}
	
	ImgProc(test);
	
	cv::threshold(test, test, 240, 255, CV_THRESH_BINARY);
	
	cv::imshow("Input EVG", test);
	
	AASS::Thinker_EVG tv;
	
	tv.setPruning(false);
	
	tv.think(test);
	cv::imshow("graph EVG", tv.getResult());
	
	AASS::topologicalmap::MultipleLineFollower mlf;
	mlf.setD(2);
	mlf.inputMap(tv.getResult());
	mlf.thin();
	std::cout << "DONE MAKING GRAPH" << std::endl;
	return mlf.getGraph(0);
	
}


AASS::topologicalmap::GraphList getGraphListVoronoi(cv::Mat& testin){
	

	cv::Mat testt;
	testin.copyTo(testt);
	
	cv::threshold(testt, testt, 240, 255, CV_THRESH_BINARY);
	
	ImgProc(testt);
	
	cv::Mat test =  cv::Scalar::all(255) - testt;
	
	cv::imshow("input voronoi", testt);
				
	AASS::vodigrex::ThinkerVoronoi tv;
	tv.think(test);
	cv::Mat res = tv.getResult();
	
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
	for(int i = 0 ; i < 1 ; ++i){
		cv::dilate(res, res, kernel);
	}
	
	cv::imshow("graph Vronoi", res);
	
	AASS::topologicalmap::MultipleLineFollower mlf;
	mlf.setD(2);
	mlf.inputMap(res);
	mlf.thin();
	std::cout << "DONE MAKING GRAPH" << std::endl;
	return mlf.getGraph(0);
	
}



void getGraphsList(cv::Mat& test, cv::Mat test_deux, AASS::topologicalmap::GraphList& gl, AASS::topologicalmap::GraphList& gl_deux, AASS::topologicalmap::GraphList& gl_voro, AASS::topologicalmap::GraphList& gl_deux_voro){

	
	gl.clear();
	gl_deux.clear();
	gl_voro.clear();
	gl_deux_voro.clear();
	
	std::cout << "New File test " << std::endl;	
	
	cv::imshow("MODEL", test);
	cv::imshow("INPUT", test_deux);
		
	gl = getGraphListEVG(test);
	cv::Mat graph_test = cv::Mat::zeros(test.rows , test.cols, CV_8UC3);
	gl.draw(graph_test);
	cv::imshow("evg model graph", graph_test);
	
	gl_voro = getGraphListVoronoi(test);
	cv::Mat graph_test_voro = cv::Mat::zeros(test.rows , test.cols, CV_8UC3);
	gl_voro.scale(2);
	gl_voro.draw(graph_test_voro);
	gl_voro.scale(0.5);
	cv::imshow("voro model graph", graph_test_voro);
	
	std::cout << "Doing the model now" << std::endl;
	
	gl_deux = getGraphListEVG(test_deux);
	cv::Mat graph_test_deux = cv::Mat::zeros(test_deux.rows , test_deux.cols, CV_8UC3);
	gl_deux.draw(graph_test_deux);
	cv::imshow("evg graph", graph_test_deux);
	
	gl_deux_voro = getGraphListVoronoi(test_deux);
	cv::Mat graph_test_deux_voro = cv::Mat::zeros(test_deux.rows , test_deux.cols, CV_8UC3);
	gl_deux_voro.scale(2);
	gl_deux_voro.draw(graph_test_deux_voro);
	gl_deux_voro.scale(0.5);
	cv::imshow("voro graph", graph_test_deux_voro);
	
}



int main(int argc,      // Number of strings in array argv
          char *argv[]){

	std::string path("../Test/GraphDB/SLAM_images");
	AASS::graphmatch::GraphPlace gp;
	boost::filesystem::path p(path);
	try{
		while(! boost::filesystem::exists(p)){
			std::cout << "need a valid path toward the slam images" << std::endl;
			std::cin >> path;
			p = path;
		}
		
		if(boost::filesystem::is_directory(p)){
			
			std::vector<boost::filesystem::path> v;
			//Get all files and sort them
			std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));
			std::sort(v.begin(), v.end());
			
			for (std::vector<boost::filesystem::path>::const_iterator it (v.begin()); it != v.end(); it = ++it)
			{
				boost::filesystem::path fn = *it;
				cv::Mat test_model = cv::imread(fn.string(), CV_LOAD_IMAGE_GRAYSCALE);
				++it;
				boost::filesystem::path fn_deux = *it;
				cv::Mat test = cv::imread(fn_deux.string(), CV_LOAD_IMAGE_GRAYSCALE);
	
				AASS::topologicalmap::GraphList gl_model;
				AASS::topologicalmap::GraphList gl;
				AASS::topologicalmap::GraphList gl_voro_model;
				AASS::topologicalmap::GraphList gl_voro;
				
				getGraphsList(test_model, test, gl_model, gl, gl_voro_model, gl_voro);
				
				cv::Mat graph_test_deux_voro = cv::Mat::zeros(test.rows , test.cols, CV_8UC3);
				gl_voro.scale(2);
				gl_voro.draw(graph_test_deux_voro);
				gl_voro.scale(1/2);
				cv::imshow("voro after graph", graph_test_deux_voro);
				
				cv::waitKey(0);
				
				AASS::graphmatch::GraphPlace gp_model = getGraphPlace(gl_model, test_model);
				cv::Mat graph_test_model = cv::Mat::zeros(test_model.rows , test_model.cols, CV_8UC3);
				gp_model.draw(graph_test_model);
				cv::imshow("evg gp model", graph_test_model);
				
				AASS::graphmatch::GraphPlace gp = getGraphPlace(gl, test);
				cv::Mat graph_test = cv::Mat::zeros(test.rows , test.cols, CV_8UC3);
				gp.draw(graph_test);
				cv::imshow("evg gp", graph_test);
				
				
				//BUG I DON'T KNOW WHY
				
				AASS::graphmatch::GraphPlace gp_voro = getGraphPlace(gl_voro, test);
				cv::Mat graph_test_voro = cv::Mat::zeros(test.rows , test.cols, CV_8UC3);
				gp_voro.scale(2);
				gp_voro.draw(graph_test_voro);
				gp_voro.scale(1/2);
				cv::imshow("voro gp", graph_test_voro);
				
				AASS::graphmatch::GraphPlace gp_voro_model = getGraphPlace(gl_voro_model, test_model);
				cv::Mat graph_test_voro_model = cv::Mat::zeros(test_model.rows , test_model.cols, CV_8UC3);
				gp_voro_model.scale(2);
				gp_voro_model.draw(graph_test_voro_model);
				gp_voro_model.scale(1/2);
				cv::imshow("voro gp model", graph_test_voro_model);
				
				cv::waitKey(0);
				
				
				
				
				
				
				
				
				
			}
			
			
		}
		
		
		
	}
	catch (const boost::filesystem::filesystem_error& ex)
	{
		std::cout << ex.what() << '\n';
	}
	
}