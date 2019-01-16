#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <eigen3/Eigen/Geometry>


std::string type2str(int type) {
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

	return r;
}


std::vector<std::string> getWords(const std::string& line){

	std::istringstream iss(line);
	char split_char = ' ';
	std::vector<std::string> tokens;
	for (std::string each; std::getline(iss, each, split_char); tokens.push_back(each));

	return tokens;
}

cv::Mat get_affine_from_file(const std::string& file){


//	cv::Mat affine( 2, 3, CV_32FC1 );
	std::ifstream infile(file);
	std::string line;
	std::vector<float> values;
	int count = 0;
	while (std::getline(infile, line) && count < 2) {
		auto words = getWords(line);
		for(auto el : words){
			values.push_back(atof(el.c_str()));
			std::cout << el << " " << atof(el.c_str()) <<  std::endl;
		}
		++count;
	}
	cv::Mat affine_tmp( values);
	cv::Mat affine;
	affine_tmp.reshape(0,2).copyTo(affine);
	std::cout << "Affine in function " << affine << std::endl;

//	affine_out = affine;
//	std::cout << "Affine_out in function " << affine << std::endl;
	return affine;

}



int main(int argc, char** argv) {

	auto img1 =  cv::imread("../../../../Test/RSI/test_affine/E5_11.png", CV_LOAD_IMAGE_GRAYSCALE);
//	auto img12 =  cv::imread("filename2.png", CV_LOAD_IMAGE_GRAYSCALE);

	std::string file = "../../../../Test/RSI/test_affine/E5_11.txt";
//	cv::Mat affine_out;
	cv::Mat affine = get_affine_from_file(file);

//	std::cout << "Affine " << affine_out << " type " << affine_out.type() << std::endl;
	std::cout << "Affine\n" << affine << "\ntype -> " << affine.type() << " " << type2str(affine.type()) << std::endl;

	cv::Point2f srcTri[3];
	cv::Point2f dstTri[3];

	/// Set the dst image the same type and size as src
	cv::Mat warp_dst = cv::Mat::zeros( img1.rows, img1.cols, img1.type() );

	/// Set your 3 points to calculate the  Affine Transform
	srcTri[0] = cv::Point2f( 0,0 );
	srcTri[1] = cv::Point2f( img1.cols - 1, 0 );
	srcTri[2] = cv::Point2f( 0, img1.rows - 1 );

	dstTri[0] = cv::Point2f( img1.cols*0.0, img1.rows*0.33 );
	dstTri[1] = cv::Point2f( img1.cols*0.85, img1.rows*0.25 );
	dstTri[2] = cv::Point2f( img1.cols*0.15, img1.rows*0.7 );

	/// Get the Affine Transform
	cv::Mat warp_mat = getAffineTransform( srcTri, srcTri );

	std::cout << "Affine\n" << warp_mat << "\ntype -> " << warp_mat.type() << " " << type2str(warp_mat.type()) << std::endl;

	/// Apply the Affine Transform just found to the src image
	warpAffine( img1, warp_dst, affine, img1.size() );


	cv::imshow("Source", img1);
	cv::imshow("Warped", warp_dst);
	cv::waitKey(0);

}