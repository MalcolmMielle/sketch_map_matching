#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>

#include "SketchMap.hpp"
#include "vodigrex/voronoidiagram/ThinkerVoronoi.hpp"
#include "PlaceExtractorList2Place.hpp"
BOOST_AUTO_TEST_CASE(trying)
{
	AASS::vodigrex::ThinkerVoronoi tv;
	AASS::graphmatch::PlaceExtractorList2Place pe;
	AASS::SketchMap m(tv, pe);
	
	int width=10;
	int heigh = 10;
	cv::Mat mat_test(heigh,width, CV_8UC3, cv::Scalar(0,0,0));
	cv::circle(mat_test, cv::Point2d(0, 0), 1, cv::Scalar(255,255,255), -1);
	cv::circle(mat_test, cv::Point2d(width-1, heigh-1), 1, cv::Scalar(255,255,255), -1);
	cv::circle(mat_test, cv::Point2d(0, heigh-1), 1, cv::Scalar(255,255,255), -1);
	cv::Mat result_ccomp;
	cv::Mat other_result;
	cv::Mat label;
	cv::Mat label_pixel;
	cv::cvtColor(mat_test, mat_test, CV_RGB2GRAY);
	cv::threshold(mat_test, mat_test, 10, 255, CV_THRESH_BINARY_INV);
	cv::Mat dist;
	cv::distanceTransform(mat_test,result_ccomp, label, CV_DIST_L2, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_CCOMP);
	cv::distanceTransform(mat_test, other_result, label_pixel, CV_DIST_L2, CV_DIST_MASK_PRECISE, CV_DIST_LABEL_PIXEL);
	cv::normalize(result_ccomp, result_ccomp, 0, 1., cv::NORM_MINMAX);
	cv::normalize(other_result, other_result, 0, 100, cv::NORM_MINMAX);
	//cv::normalize(label, label, 0, 100, cv::NORM_MINMAX);
	
	
	cv::MatIterator_<uchar> it, end;
	cv::MatIterator_<uchar> modif, modif_resul;
	cv::Mat lines = mat_test.clone();
	
	int value=0;
	
	int i=0;
	int flag=0;
	modif_resul = lines.begin<uchar>();
	it = label.begin<uchar>();
	/*for(int go=0; go<mat_test.size().height*mat_test.size().width; go++){
		
		std::cout << (int) (*it)<< " ";
		i++;
		std::vector<int> vec;
		modif = it - 1;
		if( (*modif) != (*it)){
			value=value+1;
		}
		modif = it + 1;
		if( (*modif) != (*it)){
			value=value+1;
		}
		modif = it - mat_test.size().width;
		if( (*modif) != (*it)){
			value=value+1;
		}
		modif = it + mat_test.size().width;
		if( (*modif) != (*it)){
			value=value+1;
		}
		
		(*modif_resul) = value;
		value = 0;
		flag++;
		modif_resul++;
		it++;
		
		if(i == width){
			i=0;
			std::cout << std::endl;
		}
	}*/
	
	cv::Mat sobelX;
	cv::Mat sobelY;
	cv::Mat sobel;
	//int ddepth = CV_16S;
	int scale = 1;
	int delta = 0;
	/// Gradient X
	label.convertTo(label,CV_32F);
	
	std::cout << "type : " << sobelX.type() << " type label : "<<label.type()<<std::endl;
	cv::Sobel( label, sobelX, -1, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT );
/// Gradient Y
	cv::Sobel( label, sobelY, -1, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT );
	cv::addWeighted( sobelX, -0.5, sobelY, -0.5, 0, sobel );
	
	
	//FAST
	
	
	
	cv::Mat sobel_erode;
	cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
	//cv::erode(sobel, sobel_erode, kernel, cv::Point(-1, -1), 10, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());
	cv::dilate(sobel, sobel_erode, kernel);
	cv::erode(sobel_erode, sobel_erode, kernel);
	
	
	cv::Mat pyr;
	cv::pyrDown(sobel, pyr);
	
	cv::Mat laplace;
	cv::Laplacian(label, laplace, -1);
	
	cv::Mat fast;
	std::vector<cv::KeyPoint> keypoints;
	
	cv::FAST(laplace, keypoints, 0);
	
	std::cout << "KEYOINTS " << keypoints.size() <<" : ";
	for(int i=0; i<keypoints.size();i++){
			std::cout <<"x : "<<keypoints[i].pt.x<<" y : "<<keypoints[i].pt.y<<" - " ;
	}
	std::cout <<std::endl;
	
	/*std::cout << "erode (python)  = " << std::endl << cv::format(kernel,"python") << std::endl << std::endl;
	
	std::cout << "pyr (python)  = " << std::endl << cv::format(pyr,"python") << std::endl << std::endl;
	
	std::cout << "laplace (python)  = " << std::endl << cv::format(laplace,"python") << std::endl << std::endl;
	
	std::cout << "sobel (python)  = " << std::endl << cv::format(sobel,"python") << std::endl << std::endl;
	
	std::cout << "sobel erode (python)  = " << std::endl << cv::format(sobel_erode,"python") << std::endl << std::endl;
	
	
	std::cout << "lines (python)  = " << std::endl << cv::format(lines,"python") << std::endl << std::endl;
	
	
	
	
	std::cout << "R (python)  = " << std::endl << cv::format(mat_test,"python") << std::endl << std::endl;
	
	std::cout << "R (python)  = " << std::endl << cv::format(label,"python") << std::endl << std::endl;
	
	std::cout << "R (python)  = " << std::endl << cv::format(label_pixel,"python") << std::endl << std::endl;
	
	std::cout << "R (python)  = " << std::endl << cv::format(other_result,"python") << std::endl << std::endl;*/
	
	/*std::string name("MapCreator");
	cv::namedWindow(name, 1);
	cv::imshow("MapCreator", mat_test);
	
	std::string name2("Voronoi");
	cv::namedWindow(name2, 1);
	cv::imshow(name2, result_ccomp);
	
	std::string name4("Voronoi_pixel");
	cv::namedWindow(name4, 1);
	cv::imshow(name4, other_result);
	
	std::string name3("Label");
	cv::namedWindow(name3, 1);
	cv::imshow(name3, label);*/
	
	std::string name5("Sobel");
	cv::namedWindow(name5, 1);
	cv::imshow(name5, sobel);
	
	cv::waitKey(0);
	
	
	
}