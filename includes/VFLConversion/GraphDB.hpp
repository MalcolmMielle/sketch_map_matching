/*
 The graph file format
 The graphs are stored in a compact binary format, one graph per file. The file is composed of 16 bit words, 
 which are represented using the so-called little-endian convention, i.e. the least significant byte of the word is stored first.
 
 Two different formats are used for labeled and unlabeled graphs. 
 In unlabeled graphs, the first word contains the number of nodes in the graph; this means that this format can deal with 
 graphs of up to 65535 nodes (however, actual graphs in the database are quite smaller, up to 1024 nodes). Then, for each node, 
 the file contains the list of edges coming out of the node itself. The list is represented by a word encoding its length, 
 followed by a word for each edge, representing the destination node of the edge. Node numeration is 0-based, so the first node 
 of the graph has index 0.The following C code shows how a graph file can be read into an adjacency matrix; the code assumes that 
 the input file has been opened using the binary mode.
 */

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <stdexcept>

#define MAX 20


inline int read_word(FILE* f){
	unsigned char bytes[2];
	if(fread(&bytes, 2, 1, f) != 0){
		return bytes[0] | (bytes[1]<<8);
	}
	return 0;
}




/**
 * @brief Read unlabel graph from ARG database
 * The return value is the number of nodes in the graph.
 * @return number of node in the graph
 */
inline int read_graph_unlabeled(FILE *in, char **matrix[])
{   
	int nodes;
    int edges;
    int target;
    int i, j;
	
    /* Read the number of nodes */
    nodes = read_word(in);
	if(nodes > 65535){
		throw std::runtime_error("TOO MANY NODES");
	}
	
// 	std::cout << "nb nodes " << nodes << std::endl;
	*matrix = (char**)malloc(sizeof(char*) * nodes);
	for(int y = 0 ; y < nodes ;y++){
		//malloc the 'x' dimension
		(*matrix)[y] = (char*)malloc(sizeof(char) * nodes);
		//iterate over the 'x' dimension
		for( int x = 0 ; x < nodes ; x++){
			//malloc the string in the data structure
			(*matrix)[y][x] = 0;
// 			std::cout << "HERE " << y << " " << x << std::endl;

		}
	}
	
	
// 	std::cout << " value " << (*matrix)[0][3] << "was ? " << std::endl;
	
    /* Clean-up the matrix */
//     for(i=0; i<nodes; i++)
// 		for(j=0; j<nodes; j++)
// 			matrix[i][j]=0;
	
    /* For each node i ... */
    for(i=0; i<nodes; i++)
	{ 
        /* Read the number of edges coming out of node i */
        edges = read_word(in);
// 		std::cout << "nb edges" << edges << std::endl;
        /* For each edge out of node i... */
        for(j = 0; j < edges; j++)
		{ 
// 			std::cout << " at " << i << " " << j << std::endl;
// 			std::cout << " valuev2 " << (*matrix)[0][3] << "was ? " << std::endl;
// 			printf("for real %d\n", (*matrix)[0][3]);
            /* Read the destination node of the edge */
            target = read_word(in);
// 			std::cout << "Target " << target << std::endl;
            /* Insert the edge in the adjacency matrix */
            (*matrix)[i][target] = 1;
		}
	}
	
// 	std::cout << " value after everything is set and done " << (*matrix)[0][3] << "was ? " << std::endl;
// 	printf("for real %d\n", (*matrix)[0][3]);
	
    return nodes;
}



/*
 The graph file format
 The graphs are stored in a compact binary format, one graph per file. The file is composed of 16 bit words, 
 which are represented using the so-called little-endian convention, i.e. the least significant byte of the word is stored first.
 
 Two different formats are used for labeled and unlabeled graphs. 
 In labeled graphs, we assume that attributes are represented by integer numbers. Also, the use of floating point values can be 
 somewhat more problematic, because:
 -	usually it makes little sense to compare floating point numbers for strict equality 
 -	there is no universally useable binary format for storing binary numbers (there are still machines or programming languages 
	that do not support IEEE 754 out there,	and converting from a floating point format to another can be quite a nightmare); 
	hence we should stick to a text format, which requires quite more space.
 
 These inconvenients are not repaid for by significant advantages, since integer attributes can also be used to perform arithmetic 
 tasks, e.g. for a distance evaluation. The most important parameter characterizing the difficulty of the matching problem is the number  N of different attribute values: obviously the higher this number, the easier is the matching problem. It is important to have in the database different values of N.
 
 In order to avoid the need to have several copies of the database with different values of N, we can exploit the following idea: 
 each (node or edge) attribute is generated as a 16 bit value, Then, we can make experiments with any N of the form 2^k, for k not 
 greater than 16, just by using, in the attribute comparison function, only the first k bits of the attribute.
 
 With this technique we made the attributed graph database that is general enough for experimenting with many different attribute 
 cardinalities, without incurring in the explosion of the size required to store the database.
 
 Following is the labeled graph file format.
 The first word contains the number of nodes in the graph; this means that this format can deal with graph of up to 65535 nodes 
 (however, actual graphs in the database are quite smaller, up to 100 nodes).
 Then follow N words, that are the attribute of corresponding node. Then, for each node i, the file contains the number of edges coming 
 out of the node itself (Ei) followed by 2*Ei words that code the edges. In particular, for each edge, the first word represents the 
 destination node of the edge and the second word represents the edge attribute. Node numeration is 0-based, so the first node of 
 the graph has index 0.
 
 The following C code shows how a graph file can be read into an adjacency matrix, a vector representing the nodes attribute and a 
 matrix representing the edges attribute; the code assumes that the input file has been opened using the binary mode
 
 */


// /* WARNING: for simplicity this code does not check for End-Of-File! */
// unsigned short read_word_labeled(FILE *in)
// { unsigned char b1, b2;
// 	
//     b1=getc(in); /* Least-significant Byte */
//     b2=getc(in); /* Most-significant Byte */
// 	
//     return b1 | (b2 << 8);
// }

/* This function assumes that the adjacency matrix is statically allocated.
 * The return value is the number of nodes in the graph.
 */
int read_graph(FILE *in, int matrix[MAX][MAX], int node_attr[MAX], int edge_attr[MAX][MAX])
{
    int nodes;
    int edges;
    int target, i, j;
	
    /* Read the number of nodes */
    nodes = read_word(in);
	
    /* Read the attributes of nodes */
    for(i=0; i<nodes; i++)
	{ node_attr[i] = read_word(in);
	}
	
    /* Clean-up the matrix */
    for(i=0; i<nodes; i++)
		for(j=0; j<nodes; j++)
			matrix[i][j]=0;
	
    /* For each node i... */
    for(i=0; i<nodes; i++)
	{
		/* Read the number of edges coming out of node i */
		edges=read_word(in);
		
		/* For each edge out of node i... */
		for(j=0; j<edges; j++)
		{
			/* Read the destination node of the edge */
			target = read_word(in);
			
			/* Insert the edge in the adjacency matrix */
			matrix[i][target] = 1;
			
			/* Read the edge attribute and insert in the matrix that holds the attributes of edges */
			edge_attr[i][target] = read_word(in);
		}
	}
	
	
    return nodes;
}

