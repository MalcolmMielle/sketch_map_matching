#include <iostream>
#include <cstdlib>
#include <string>
#include <experimental/filesystem>
#include <cassert>

int main(){

	//	std::string file;
	//std::experimental::filesystem::path p;
	std::string path = "../../../../Test/RSI/RobotMaps/E5";

	std::string model = "../../../../Test/RSI/RobotMaps/E5/model_simple.png";

	//HACK because can't copy iterator
//	int count = 0;

	auto rec = std::experimental::filesystem::directory_iterator(path);


	for (auto p = std::experimental::filesystem::begin(rec) ; p != std::experimental::filesystem::end(rec) ; ++p) {

//		auto rec2 = std::experimental::filesystem::recursive_directory_iterator(path);
//		auto p_second = std::experimental::filesystem::begin(rec2);
//		++count;
//		for(int i = 0 ; i < count ; ++i){
////			std::cout << "Increment" << std::endl;
//			assert(p_second != std::experimental::filesystem::end(rec2));
//			++p_second;
//		}
//		for ( ; p_second != std::experimental::filesystem::end(rec2) ; ++p_second) {
			auto p_canon = std::experimental::filesystem::canonical(*p);
			std::cout << p_canon << std::endl; // "p" is the directory entry. Get the path with "p.path()"
//			auto p_canon_second = std::experimental::filesystem::canonical(*p_second);
//			std::cout << p_canon_second << std::endl; // "p" is the directory entry. Get the path with "p.path()"

			std::string program = "/home/malcolm/AASS/sketch_algorithms/build/Test/RSI/Evaluation/create_correspondences ";
			std::string file_export = "gt_" + p_canon.stem().string() + "_model_simple.dat";
//			program.append(p_canon.string() + " " + p_canon_second.string() + " " + file_export);
			program.append(p_canon.string() + " " + model + " " + file_export);

			std::cout << "Launching " << program << std::endl;

			system(program.c_str());

			std::cout << "DONE " << std::endl;
//			exit(0);
//		}

//		std::cout << "NEW" << std::endl;
	}

}
