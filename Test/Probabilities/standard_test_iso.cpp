#include <iostream>
#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include <fstream>
#include <ctime>
//TODO : test with empty graph

#include "GraphList.hpp"
#include "argraph.h"
#include "argedit.h"
#include "vf2_sub_state.h"
#include "match.h"
#include "argloader.h"

#define MAXNODES 200

// #include "Graph.hpp"

class PointWeird{
public :
	float x, y;
	PointWeird(float xx, float yy){
		x = xx;
		y = yy;
	}
	
};

template <typename T>
class PointWeirdAllocator : public Allocator<T>{
	protected:
      virtual T* alloc() { return new T(0,0); }
};

istream& operator>>(istream& in, PointWeird &p){
	in >> p.x >> p.y;
	return in;
}

ostream& operator<<(ostream& in, PointWeird &p){
	in << p.x << " " << p.y;
	return in;
}

class Empty{
	
};

istream& operator>>(istream& in, Empty &p){
	
	return in;
}

ostream& operator<<(ostream& in, Empty &p){
	
	return in;
}

class PointDestroyer : public AttrDestroyer {
	
public :
	virtual void destroy(void *p){
		delete p;
	}
};

class PointComparator : public AttrComparator{

public:
	double threshold;
	
	PointComparator(double th){
		threshold = th;
	}
	
	virtual bool compatible(void *pa, void *pb){
		PointWeird *a = (PointWeird *)pa;
		PointWeird *b = (PointWeird *)pb;
		double dist = hypot(a->x - b->x, a->y - b->y);
		return dist < threshold;
	}
	
	
};


//Visitor

bool my_visitor(int n, node_id ni1[], node_id ni2[], void *usr_data){
	
	std::cout << std::endl << "NEW MATCHING " << std::endl;
	for(int i = 0 ; i < n ; ++i) {
		std::cout << "Node " << ni1[i] << " is paired with node " << ni2[i] << std::endl;
	}
	return false;
}



BOOST_AUTO_TEST_CASE(trying)
{
	
	AASS::topologicalmap::GraphList gl;
	
	ARGEdit ed;
	ARGEdit ed_g2;
	
	int i,j;
	for(i = 0 ; i < 4 ; i++){
		ed.InsertNode(NULL);
		ed_g2.InsertNode(NULL);
	}
	for(i = 0 ; i < 4 ; i++){
		for(j = 0 ; j < 4 ; ++j){
			ed.InsertEdge(i, j, new PointWeird(1,85));
			ed_g2.InsertEdge(i, j, new PointWeird(101, 102));
		}
	}
	
	ARGraph<Empty, PointWeird> g(&ed);
	ARGraph<Empty, PointWeird> g2(&ed_g2);
	
// 	g.SetNodeDestroyer(new PointDestroyer());
// 	g2.SetNodeDestroyer(new PointDestroyer());
	
	g.SetNodeComparator(new PointComparator(0.1));
	//Init the graph matching
	VF2SubState s0(&g, &g2);
	
	//Matching getting only first matching
	
// 	if(!match(&s0, my_visitor)){
		
// 		std::cout << "No matching found" << std::endl;
		
// 	}
	
	//Save graph
	
	ofstream out("graph.txt");
	StreamARGLoader<Empty, PointWeird>::write(out, g);
	out.close();
	

// 	Load a garph from file
	
// 	PointWeirdAllocator<PointWeird> node_allocator;
// 	NewAllocator<Empty> edge_allocator;
// 	
// 	ifstream in("graph.txt");
// 	StreamARGLoader<PointWeird, Empty> loader(&node_allocator, &edge_allocator, in);
// 	
// 	ARGraph<PointWeird, Empty> loaded_graph(&loader);
// 	loaded_graph.SetNodeDestroyer(new PointDestroyer());
// 	loaded_graph.SetNodeComparator(new PointComparator(0.1));
// 	
// 	VF2SubState s0_load(&loaded_graph, &g2);
// 	//Matching getting only first matching
// 	int n_load;
// 	
// 	std::cout << std::endl << std::endl << "Second test " << std::endl;
// 	if(!match(&s0_load, my_visitor)){
// 		
// 		std::cout << "No matching found" << std::endl;
// 		
// 	}
	
	/*** TEST OF MY GRAPH***/
// 	typedef boost::adjacency_list<
// 			boost::listS, boost::listS, boost::undirectedS> Graph_places;
// 			
// 			
// 	AASS::sketch_algo::Graph<Graph_places> g_mine;
	
}