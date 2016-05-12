Hello to you who stumbled upon the Sketch Maker program ! I really should change its name because it's more the _sketch algo_ but on Gitlab I don't know how to do it ;). 

# What is the Sketch Maker ?

The Sketch Maker is 2 things : 

* An interpretor of the said sketch i.e transcription from drawing to topological map

* Lots of code

# How do I use the Sketch Maker ?

To use the sketch maker you need to compile using `cmake ..` in a build directory and then `make` (`make -j4` if you have 4 cores). Next launch a bunch of test would have been create and you can test the algorithm.

# What are the dependencies of Sketch Maker ?

Sketch maker is based on _OpenCV_ (v : 2.4.9) and Boost.

Under fedora those are the command needed to install those packages :

	dnf install git cmake
	dnf install opencv-devel
	dnf install qt4-devel
	dnf install boost-devel
	
Under Ubuntu just :

	sudo apt-get install git cmake
	sudo apt-get install libopencv-dev 
	sudo apt-get install libboost-all-dev 
	
# Install

git clone the repo.

Then :


	mkdir build
	cd build
	cmake ..
	make
	

For debugging


	mkdir debug
	cd debug
	cmake -DCMAKE_BUILD_TYPE=Debug ..
	make


# I don't like the build in interpretor what can I do ?

Well, you could code your own. Every interpretor is based out of two abstract classes : 

* Thinker : first step of interpretation of the map. Sort of a preprocessing. I know, it's not the clearest name.

* PlaceExtractor : extract a GraphPlace from a GraphList.

* GraphMatcherBase : match and compare two Graphplaces.

And the non abstract class : 

* LineFollower. The line follower is the class that extract the graph from an image.

For now, as intersection types in the GraphLine, T crossing, X crossing and N (meaning any other number) crossing exist. The graph structure is stored as a _Boost Graph_. See [this](http://www.boost.org/doc/libs/1_58_0/libs/graph/doc/) for more information on the library. In particular one can use a lot of alogorithm like Dikjstra or Astar. The graph extracted from the lines is a 

	typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, topologicalmap::Intersection_Graph, boost::edge_weight_t, boost::no_property >

meaning that it's stored in vectors, it is undirected and the vertices are a `Intersection_Graph` object defined as :

	struct Intersection_Graph{
		std::string type;
		cv::Mat mat;
		cv::Point2i point;
		int index;
	};

For now, the edge got all the same weight but in the future an approximate distance might be added.

For the comparison a GraphPlace is used :

	typedef boost::adjacency_list<boost::listS, boost::listS, boost::undirectedS, graphmatch::Place, graphmatch::Gateway_struct, boost::no_property > Graph_places;
	
It's stored in lists, it's undirected, the vertices are `Place` objects and the Edges and `Gateway_struct`.

`Place` is an important object since they include an element named `Keypoint` which is used for the comparison. It's easy to had new types of Place and different comparison methods thanks to their Keypoint elements.

## The Keypoint class :

### Create the Keypoints

The Keypoint class is the object used for every comparison between two graph places. It's fairly easy to had new type of keypoints and comparison method. Just follow those steps :
 
1. Create every class of vertex you want to use for the comparison algorithm. Every of those class must inheritate from `Keypoint` and reimplement the following functions :

	* Keypoint(const std::string& typet) : type(typet){}
	Keypoint() : type("notype"){ std::cout << "Using the constructor" << std::endl;}
			
	* //@brief return a one character representing the type
	virtual std::string getID() const 

	* ///@brief return true if the graphmatch::VertexPlace is of *this type
	virtual bool isOfType(const graphmatch::VertexPlace& v)

	* ///@brief return a pointer to an element on this class if the vertex is determine to be of this class.
	virtual Keypoint* compare(const graphmatch::VertexPlace& v, const graphmatch::Graph_places& gp) const{

	* virtual Keypoint* makePointer() const 

	* virtual cv::Scalar getColor(int channel) const
	
For every keypoint you want to use :

* Your keypoint types must define the whole space.

* No keypoint should return the same character ID

See `JunctionAndDeadEnds.hpp` for an example.


### Create a Place extractor :

You then have to implement a class that inheritate from PlaceExtractorBase. The function that extract a GraphPlace with your keypoints from a GraphList is name `extract` and is the only function that you must implement. The results are stored in `graphmatch::GraphPlace _graph`.

Every PlaceExtractor should be associated with a class inheritating from `AllKeypoints`. The only function one would need to implement is `init_all_types_of_keypoints()` were you need to add a list of `Keypoint` pointers to the `_all_types_of_keypoints` attribute of the class.

This class is used during the place extraction. Mainly, the function `compare` is used to see if one of the `Keypoint` stored in the onject inheritating from `AllKeypoints` correspond to the vertex inputed. If yes, a new pointer to an Object of the same Keypoint is return and must be stored in the input vertex. If non, a pointer to `NULL` is returned.

Now all this must seem complicated and a lot of work __but__ If you only want to use an existing extraction method with your custom keypoints, it's really easy. All you have to doo is declare your keypoints and then change the `AllKeypoints` element in the place extractor you want to use. Simple as that ! :)


# How does the interpretor works ?

For now the interpretation is made like this : 

* First, the drawing is processed and the voronoi lines are extracted. This result in an image where the voronoi lines are white and the rest is black. There is multiple ways to obtain the voronoi line but the most efficient one (as in fast and accurate) is to use LaPlace on a trimmed version of a map of all the distance, with a downsampling coef of 1. Like this it's stil possible to draw in Real Time. See next section for more detail.

* Second, the line are processed using a slightly modified version of the Algorithm presented in __Orit BARUCH - Line Thinning by line following - Pattern Recognition Letters 8 1988__. By line following we detect the intersections and create a Topological map out of this. The main advantage of this technique is that the algorithm can be reused with any given graph. So if the user want to draw a path it can be interpreted. It's not only a matter of drawing walls.

The main advantage of doing all of this graphically is the possibility to process a _picture of a map_. Just by having a camera, a picture of sketch could be sent to the robot for interpretation. The intrepretation is entirerly _interface independant_.

* Then the places in the map are extracted by PlaceExtractor and converted to a GraphPlace.

* Finally a GraphMatcher compare to SketchMap to figure out the best matching.

# How are the voronoi lines extracted ?

Although, several method to extract Voronoi Lines exists as the [EVG Thin](http://openslam.informatik.uni-freiburg.de/evg-thin.html), I found this method to be more efficient and faster. Plus, it does not remove some of the lines that are removed by EVG. The only problem is that there is a lot of noise when using it a sketch maps. Indeed, lots of curvature == lots of lines.

## If you have a perfect building map :

By perfect, we consider all building map with straight corridors. The method is as such :

* Compute the pixel distance of every point in the image.

* Convolve the distance image with either [0, 1, 0 ; 1 , -4, 1 ; 0, 1, 1] or [1, 1, 1 ; 1 , -8, 1 ; 1, 1, 1].

* The negative values in the resulting image correspond to local maximas in the distance image and are the voronoi lines.

## If the map is a sketch :

* Compute the pixel distance of every point in the image.

* Blur it using a 5*5 gaussian kernel and downsample it by removing every even rows and columns. This step is needed to remove all the weak maximas.

* Convolve the distance image with either [0, 1, 0 ; 1 , -4, 1 ; 0, 1, 1] or [1, 1, 1 ; 1 , -8, 1 ; 1, 1, 1].

* Keep the 70% most negative value of the image (the strong maximas of the distance image).

This is the same method as presented in `Parameter controlled skeletonization of three dimensional objects` by `Gagvani`. Only, in his version the "thinness parameter" (which basically is our percentage) is manually chosen while here, we systematically use 70%.
