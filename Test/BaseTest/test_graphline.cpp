#define BOOST_TEST_DYN_LINK

#define BOOST_TEST_MODULE MyTest
#include <time.h>
#include <cstdlib>
#include <boost/test/unit_test.hpp>
#include "GraphLine.hpp"

BOOST_AUTO_TEST_CASE(trying)
{
	

	bettergraph::PseudoGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge> graph;
	
	AASS::topologicalmap::NodeLine sn;
	sn.setPoint(1, 2);
	bettergraph::PseudoGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge>::Vertex vert;
	bettergraph::PseudoGraph<AASS::topologicalmap::NodeLine, AASS::vodigrex::SimpleEdge>::Vertex vert2;
	graph.addVertex(vert, sn);
	AASS::topologicalmap::NodeLine sn2;
	sn.setPoint(1, 5);
	graph.addVertex(vert2, vert, sn2);
	
	AASS::topologicalmap::GraphLine gl = AASS::topologicalmap::GraphLine(graph);
	
	BOOST_CHECK_EQUAL(graph.getNumVertices(), 2);
	BOOST_CHECK_EQUAL(graph.getNumEdges(), 1);
	BOOST_CHECK_EQUAL(graph[vert].getX(), 1);
	BOOST_CHECK_EQUAL(graph[vert].getY(), 2);
	
	BOOST_CHECK_EQUAL(gl.getNumVertices(), 2);
	BOOST_CHECK_EQUAL(gl.getNumEdges(), 1);
	BOOST_CHECK_EQUAL(gl[vert].getX(), 1);
	BOOST_CHECK_EQUAL(gl[vert].getY(), 2);
	
	
	AASS::topologicalmap::GraphLine::VertexLine vert3;
	AASS::topologicalmap::NodeLine sn3;
	sn3.setPoint(10, 50);
	gl.addVertex(vert3, vert, sn3);
	
	BOOST_CHECK_EQUAL(gl.getNumVertices(), 3);
	BOOST_CHECK_EQUAL(gl.getNumEdges(), 2);
	BOOST_CHECK_EQUAL(gl[vert3].getX(), 10);
	BOOST_CHECK_EQUAL(gl[vert3].getY(), 50);
	
	BOOST_CHECK_EQUAL(graph.getNumVertices(), 2);
	BOOST_CHECK_EQUAL(graph.getNumEdges(), 1);
}