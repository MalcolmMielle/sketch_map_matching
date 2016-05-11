#include "GraphDB.hpp"
#include "GraphPlace.hpp"

#include <boost/filesystem.hpp>
#include <boost/iterator/filter_iterator.hpp>

#include <boost/filesystem/path.hpp>

#include "Timed.hpp"
#include "GraphMatcherNeighbor.hpp"
#include "ConversionVFL.hpp"
#include "match.h"
#include "vf2_sub_state.h"
#include "argloader.h"

char getGraphSize(boost::filesystem::path fn, int& nodes){
	std::string ext = fn.filename().stem().string();

	int count = 0;
	while(count < 2 && ext.size() > 0){
		char lim = '_';
// 		std::cout <<< ext << " size " << ext.size() << " core " << strcmp(&lim[0], &ccc) << std::endl;
		if(lim == ext[0]){
			++count;
		}
		if (ext.size () > 0)  ext.erase(0, 1);
	}
	if(ext.size() == 0){
		std::cout << "PROBLEM" << std::endl;
		exit(0);
	}
	char size =  ext[0];
	if (ext.size () > 0)  ext.erase(0, 1);
	
	char *end;
	nodes = strtol(ext.c_str(), &end, 0);
	
	return size;	
}


std::string getExtensionLetter(boost::filesystem::path fn){
	std::string ext = fn.extension().string();
	if (ext.size () > 0)  ext.resize (ext.size () - 1);
	if (ext.size () > 0)  ext.resize (ext.size () - 1);
	return ext;	
}

std::string getExtensionNumber(boost::filesystem::path fn){
	std::string ext = fn.extension().string();
	ext.erase(0, 2);
	return ext;
}

std::string getName(boost::filesystem::path fn){
	std::string ext = fn.extension().string();
	if (ext.size () > 0)  ext.resize (ext.size () - 1);
	if (ext.size () > 0)  ext.resize (ext.size () - 1);
	return ext;	
}


void readGraph(const std::string& file1, AASS::graphmatch::GraphPlace& gp1){
//working on iso_m2D_m196.A00
	FILE *in1 = fopen(file1.c_str(), "rb");
	
	char** matrix1;
	int nodes1 = read_graph_unlabeled(in1, &matrix1);
	
// 	std::cout << " value " << matrix[0][3] << std::endl;
// 	std::cout << "NODES" << nodes << std::endl;
	gp1.read(matrix1, nodes1);
	
	for(int y = 0 ; y < nodes1 ;y++){
		//malloc the 'x' dimension
		free(matrix1[y]);
		//iterate over the 'x' dimension
		
	}
	free(matrix1);
	
}


void addRandomDistance(AASS::graphmatch::GraphPlace& gp){
	
	int count = 0 ;
	
	std::pair<AASS::graphmatch::VertexIteratorPlace, AASS::graphmatch::VertexIteratorPlace> vp;
	for (vp = boost::vertices(gp.getGraph() ); vp.first != vp.second; ++vp.first) {
			AASS::graphmatch::VertexPlace v = *vp.first;
			
			gp[v].mass_center.x = count;
			gp[v].mass_center.y = count;
			++count;
	}
	
}


bool my_visitor(int n, node_id ni1[], node_id ni2[], void *usr_data){
	
	std::cout << std::endl << "NEW MATCHING " << std::endl;
	for(int i = 0 ; i < n ; ++i) {
		std::cout << "Node " << ni1[i] << " is paired with node " << ni2[i] << std::endl;
	}
	return false;
}


AASS::graphmatch::Hypothese matchTest(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model){
	
	
	AASS::graphmatch::GraphMatcherNeighbor gm;
	
	std::ostringstream str_test;
	str_test <<  "test graphdb, line" << __LINE__ << " in file " << __FILE__;
	timed(str_test.str(), boost::bind( &AASS::graphmatch::GraphMatcherNeighbor::planarEditDistanceAlgorithm, &gm, boost::ref(gp), boost::ref(gp_model) ) );
	
	std::deque < AASS::graphmatch::Hypothese > res = gm.getResult();
	gm.sort(res);
	return res[0];
	
	
}


AASS::graphmatch::Hypothese matchTest(ARGraph<AASS::graphmatch::Place, AASS::Empty> graph_vfl, ARGraph<AASS::graphmatch::Place, AASS::Empty> graph_vfl_model){
	
	VF2SubState s0_load(&graph_vfl, &graph_vfl_model);
	int n_load;
	
	std::cout << std::endl << std::endl << "Second test " << std::endl;
	if(!match(&s0_load, my_visitor)){
		std::cout << "No matching found" << std::endl;
	}
	
	
}

int main(int argc,      // Number of strings in array argv
          char *argv[]   // Array of command-line argument strings
		 ){
	
	std::string path;
	if(argc == 2){
		path = argv[1];
	}
	else{
		std::cout << "need path " << std::endl;
		std::cin >> path;
	}
	
	AASS::graphmatch::GraphPlace gp;
	boost::filesystem::path p(path);
	try{
		while(! boost::filesystem::exists(p)){
			std::cout << "need a valid path " << std::endl;
			std::cin >> path;
			p = path;
		}
		
		if(boost::filesystem::is_directory(p)){
			
			std::vector<boost::filesystem::path> v; 
			 
// 			std::cout << p << " is a directory containing:\n";
// 			boost::filesystem::directory_iterator dir_first(p), dir_last;
// 			std::copy(dir_first, dir_last, std::ostream_iterator<boost::filesystem::directory_entry>(std::cout, "\n")); //Cppy the element from first to last into the range output_iterator
			
			std::copy(boost::filesystem::directory_iterator(p), boost::filesystem::directory_iterator(), std::back_inserter(v));

			std::sort(v.begin(), v.end());             // sort, since directory iteration
													// is not ordered on some file systems
			
			for (std::vector<boost::filesystem::path>::const_iterator it (v.begin()); it != v.end(); ++it)
			{
				boost::filesystem::path fn = *it;   // extract the filename from the path
// 				std::cout << "new name at " << icon << " " << std::endl;
// 				std::cout << fn.filename().stem() << std::endl;
// 				std::cout << fn.extension() << std::endl;
				std::string ext = getExtensionLetter(fn);
				
				if(ext == ".A"){
					
					int size_graph = 0;
					char c = getGraphSize(fn, size_graph);
					
					//We only consider small graphs
					if( c == 's' && size_graph < 60){
						
	// 					std::cout << "VALID WITH " << fn.filename() << std::endl;
						std::string numA = getExtensionNumber(fn);
						std::string nameA = fn.filename().stem().string();
						
						for (std::vector<boost::filesystem::path>::const_iterator it (v.begin()); it != v.end(); ++it)
						{
							boost::filesystem::path fn_2 = *it;
							std::string nameB = fn_2.filename().stem().string();
							
							if(nameB == nameA){
							
								ext = getExtensionLetter(fn_2);
								
								if(ext == ".B"){
									std::string num = getExtensionNumber(fn_2);
									if(num == numA){
										
										std::cout << "VALID WITH " << fn.filename() << " and " << fn_2.filename() << std::endl;
										AASS::graphmatch::GraphPlace gp;
										readGraph(fn.string(), gp);
										addRandomDistance(gp);
// 										
										AASS::graphmatch::GraphPlace gp_model;
										readGraph(fn_2.string(), gp_model);
										addRandomDistance(gp_model);
										
										
										AASS::VFLGraph graph_vfl = AASS::graphPlace2VFL(gp);
										AASS::VFLGraph graph_vfl_model = AASS::graphPlace2VFL(gp_model);
// 										
										matchTest(gp, gp_model);
										
										matchTest(graph_vfl, graph_vfl_model);
// 										
// 										gp.print();
// 										std::cout << std::endl;
										
									}
								}
							}
						}
					}					
					
					
				}


			}
			
		}
		else{
			std::cout << "not a directory" << std::endl;
		}
	}
    
	catch (const boost::filesystem::filesystem_error& ex)
	{
		std::cout << ex.what() << '\n';
	}
	
	return 0;
}