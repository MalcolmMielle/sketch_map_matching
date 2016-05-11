
#include "opencv2/opencv.hpp"
#include "vodigrex/linefollower/LineFollower.hpp"

int main(){
	
	cv::Mat input_robot=cv::imread("../Test/SLAM/Sequences/Seqmc20/0062.jpg", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat input_sketch=cv::imread("../Test/SLAM/Sequences/sketch.png", CV_LOAD_IMAGE_GRAYSCALE);
	cv::Mat sketch;
	cv::Mat robot;
	cv::threshold(input_robot, input_robot, 100, 255, cv::THRESH_BINARY);
	cv::threshold(input_sketch, input_sketch, 100, 255, cv::THRESH_BINARY);
	cv::GaussianBlur(input_sketch, sketch, cv::Size(3,3), 20, 20);
	
	cv::GaussianBlur(input_robot,robot, cv::Size(3,3), 20, 20);
	cv::sepFilter2D(robot, robot, sketch.depth(), 2, 2);
	
	cv::imshow("Input robot", input_robot);
	cv::imshow("Input sketch", input_sketch);
	cv::imshow("SKETCH", sketch);
	cv::imshow("ROBOT", robot);
	cv::waitKey(0);
	
	int thre = 100;
	
	for(size_t i = 0 ; i < 100 ; ++i){
		
		
		cv::GaussianBlur(sketch, sketch, cv::Size(7,7), 10, 10);
		
		cv::sepFilter2D(robot, robot, sketch.depth(), 10, 10);
		cv::GaussianBlur(robot, robot, cv::Size(11,11), 5, 5);
		
// 		cv::threshold(robot, robot, 10, 255, cv::THRESH_BINARY);
		
		
		cv::Mat copy;
		sketch.copyTo(copy);
		cv::threshold(copy, copy, thre, 255, cv::THRESH_BINARY);
		if(thre > 10){
			thre = thre - 5;
		}
		
		cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3,3), cv::Point(-1, -1) );
		for(int i = 0; i<2 ; i++){
			cv::erode(copy, copy, kernel);
		}
		
		AASS::vodigrex::LineFollower lf;
		lf.setD(10);
		cv::imshow("copy before line follower", copy);
		lf.inputMap(copy);
		lf.thin();
		copy = lf.getResult();
		
// 		cv::dilate(sketch, sketch, kernel);
		
// 		for(int i = 0; i<3 ; i++){
// 			cv::erode(robot, robot, kernel);
// 		}
// 		cv::dilate(robot, robot, kernel);
// 		for(int i = 0; i<3 ; i++){
// 			cv::erode(robot, robot, kernel);
// 		}

		cv::imshow("Input robot", input_robot);
		cv::imshow("Input sketch", input_sketch);
		cv::imshow("SKETCH", sketch);
		cv::imshow("SKETCHCopy", copy);
		cv::imshow("ROBOT", robot);
		cv::waitKey(0);
	}
	return 1;
	
}