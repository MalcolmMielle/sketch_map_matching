
#include "SketchMap.hpp"


int main(int argc, char** argv)
{
	
	cv::Mat in = cv::imread("test_laplace.png", CV_LOAD_IMAGE_COLOR);	
	cv::Mat gray;
	
	cv::cvtColor(in, gray, CV_RGB2GRAY);
	std::cout << gray << std::endl<<std::endl;
	
	cv::Laplacian(gray, gray, -1);
	
	std::cout << gray;
}