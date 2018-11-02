#include "maoris/ZoneExtractor.hpp"
#include "maoris/FuzzyOpening.hpp"
//#include "maoris/Kmean.hpp"
#include "maoris/ZoneReducer.hpp"
#include "maoris/Segmentor.hpp"

#include "maoris/ZoneExtractor.hpp"
#include "maoris/FuzzyOpening.hpp"
//#include "maoris/Kmean.hpp"
#include "maoris/ZoneReducer.hpp"
#include "maoris/Segmentor.hpp"

#include "RSI/GraphZoneRI.hpp"

#include "MapComparator/Hypothese.hpp"
#include "MapComparator/GraphMatcherNeighbor.hpp"
#include "LaplacianGraphMatching/GraphLaplacian.hpp"
#include "LaplacianGraphMatching/MatchLaplacian.hpp"

#include "RSI/GraphZoneRI.hpp"
#include "RSI/ZoneCompared.hpp"
#include "RSI/hungarian/hungarian.h"
#include "RSI/HungarianMatcher.hpp"

#include "RSIConversion.hpp"

#include "Timed.hpp"
//#include "GraphMatcherNeighbor.hpp"
#include "ConversionVFL.hpp"
#include "match.h"
#include "vf2_sub_state.h"
#include "argloader.h"

#include "VFLConversion/VFLVisitor.hpp"


class RegionDestroyer : public AttrDestroyer{
public:
	virtual void destroy(void* p){
		delete p;
	}
};


class RegionComparatorHeatAnchor : public AttrComparator{
private:
	double _threshold;
public:
	RegionComparatorHeatAnchor(double th) : _threshold(th){}

	virtual bool compatible(void *pa, void *pn){
		AASS::graphmatch::Region *a = (AASS::graphmatch::Region *)pa;
		AASS::graphmatch::Region *b = (AASS::graphmatch::Region *)pn;

		double dist = std::abs(a->getHeatAnchors() - b->getHeatAnchors());
		return dist < _threshold;

	}
};

class RegionComparatorHeat : public AttrComparator{
private:
	double _threshold;
public:
	RegionComparatorHeat(double th) : _threshold(th){}

	virtual bool compatible(void *pa, void *pn){
		AASS::graphmatch::Region *a = (AASS::graphmatch::Region *)pa;
		AASS::graphmatch::Region *b = (AASS::graphmatch::Region *)pn;

		double dist = std::abs(a->getHeat() - b->getHeat());
		return dist < _threshold;

	}
};

inline std::ostream& operator<<(std::ostream& os, const bettergraph::MatchComparable<AASS::graphmatch::Region>& dt){
	os << "Vertex : " << dt.getFirst() << " " << dt.getSecond() << std::endl;
	return os;
}



void drawHypo(bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > hypothese, AASS::graphmatch::GraphLaplacian& gp_real, AASS::graphmatch::GraphLaplacian& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, bool invert, double scale) {
	std::cout << "INvert is " << invert << std::endl;
	cv::Mat obst_copy;
	obstacle.copyTo(obst_copy);

	cv::Mat obst_model_copy;
	obstacle_model.copyTo(obst_model_copy);

	cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
	cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
	cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);

	int cols_max = obst_model_copy.size().width;
	if (cols_max < obst_copy.size().width) {
		cols_max = obst_copy.size().width;
	}

	cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
	cv::Mat all = cv::Mat::zeros(size, CV_8UC3);
	cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
	cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);

	cv::Mat roi = all(cv::Rect(0, 0, obst_copy.size().width, obst_copy.size().height));
	cv::Mat roi_linked = only_linked(cv::Rect(0, 0, obst_copy.size().width, obst_copy.size().height));
	cv::Mat roi_model = all(
			cv::Rect(0, obst_copy.size().height, obst_model_copy.size().width, obst_model_copy.size().height));

	cv::Mat roi_maps = all_maps(cv::Rect(0, 0, obst_copy.size().width, obst_copy.size().height));
	cv::Mat roi_model_maps = all_maps(
			cv::Rect(0, obst_copy.size().height, obst_model_copy.size().width, obst_model_copy.size().height));

	obst_copy.copyTo(roi_maps);
	obst_model_copy.copyTo(roi_model_maps);

	cv::Scalar color;

	if (draw_links.channels() == 1) {
		color = 255;

	} else if (draw_links.channels() == 3) {
		color[0] = 500;
		color[1] = 500;
		color[2] = 500;
	}

	cv::Scalar color_model;

	if (draw_links.channels() == 1) {
		color_model = 100;
	} else if (draw_links.channels() == 3) {
		color_model[1] = 100;
		color_model[2] = 100;
		color_model[3] = 100;
	}

	cv::Scalar color_one;

	if (draw_links.channels() == 1) {
		color_one = 255;
	} else if (draw_links.channels() == 3) {
		color_one[1] = 255;
		color_one[2] = 255;
		color_one[3] = 255;
	}

	cv::Scalar color_link_maps;

	if (all_maps.channels() == 1) {
		color_link_maps = 255;
	} else if (all_maps.channels() == 3) {
		color_link_maps[0] = 255;
		color_link_maps[1] = 255;
		color_link_maps[2] = 255;
	}

	//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
//	gp_real.scale(scale);
//	gp_model.scale(scale);


	gp_real.drawSpecial(roi);
	gp_model.drawSpecial(roi_model);


	for (size_t i = 0; i < hypothese.size(); i++) {


		cv::line(draw_links, hypothese[i].getFirst().getCenter(), hypothese[i].getSecond().getCenter(), color);
		cv::Point2i model_normal;
		cv::Point2i model;

		if (invert == false) {
			//UGLY HACK BECAUSE THE SCALING DOESN'T WORK
			model_normal = hypothese[i].getFirst().getCenter();

			model = hypothese[i].getSecond().getCenter();
			model.y = model.y + obst_copy.size().height;
		} else {

			//UGLY HACK BECAUSE THE SCALING DOESN'T WORK

			model_normal = hypothese[i].getFirst().getCenter();
			model_normal.y = model_normal.y + obst_copy.size().height;

			model = hypothese[i].getSecond().getCenter();
		}

		cv::circle(draw_links, model, 10, color_model, 3);
		cv::circle(draw_links, model_normal, 10, color_one, 3);

		cv::line(all, model_normal, model, color, 5);

		cv::Scalar color_all_linked;
		cv::RNG rrng(12345);
		if (all.channels() == 1) {
			color_all_linked = rrng.uniform(50, 255);
		} else if (all.channels() == 3) {
			color_all_linked[1] = rrng.uniform(50, 255);
			color_all_linked[2] = rrng.uniform(50, 255);
			color_all_linked[3] = rrng.uniform(50, 255);
		}

		cv::circle(all, model, 10, color_all_linked, 3);
		cv::circle(all, model_normal, 10, color_all_linked, 3);

		//cv::line(all_maps, model_normal, model, color);
		cv::line(only_linked, model_normal, model, color);

		cv::circle(only_linked, model_normal, 10, color_one, -1);
		cv::line(all_maps, model_normal, model, color_link_maps, 5);
		cv::circle(all_maps, model_normal, 10, color, -1);


		cv::circle(only_linked, model, 10, color_model, -1);
		cv::line(all_maps, model_normal, model, color_link_maps, 5);
		cv::circle(all_maps, model, 10, color, -1);

	}

	cv::imshow("ALL", all);

}


int placeWhereIsVertex(const std::vector<AASS::graphmatch::GraphLaplacian::VertexLaplacian>& vp_vector, const AASS::graphmatch::GraphLaplacian::VertexLaplacian& vp){
	for(size_t i = 0 ; i < vp_vector.size() ; ++i){
		if(vp == vp_vector[i]){
			return i;
		}
	}
	return -1;
}


/**
	 * @brief Convert GraphPlace to a AASS::VFLGraph VFL graph
	 */
ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graphLaplacian2VFL(const AASS::graphmatch::GraphLaplacian& gp){

	std::vector<AASS::graphmatch::GraphLaplacian::VertexLaplacian > vp_vector;

	ARGEdit vfl_maker;



	/* Construct the graph here*/
	std::pair<AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian, AASS::graphmatch::GraphLaplacian::VertexIteratorLaplacian> vp;
	//vertices access all the vertix
	for (vp = boost::vertices(gp.getGraph() ); vp.first != vp.second; ++vp.first) {
		AASS::graphmatch::GraphLaplacian::VertexLaplacian v = *vp.first;
		vp_vector.push_back(v);
		AASS::graphmatch::Region* place = new AASS::graphmatch::Region(gp[v]);
		vfl_maker.InsertNode( place );
	}

	for (size_t j = 0 ; j < vp_vector.size() ; ++j) {

		AASS::graphmatch::GraphLaplacian::VertexLaplacian v = vp_vector[j];
		AASS::graphmatch::GraphLaplacian::EdgeIteratorLaplacian out_i, out_end;
		AASS::graphmatch::GraphLaplacian::EdgeLaplacian e;

		for (boost::tie(out_i, out_end) = boost::out_edges(v, gp.getGraph() ); out_i != out_end; ++out_i) {
			e = *out_i;
			AASS::graphmatch::GraphLaplacian::VertexLaplacian targ = boost::target(e, gp.getGraph());

			int i = placeWhereIsVertex(vp_vector, targ);
			if(i == -1){
				std::ostringstream str_test;
				str_test <<  "Vertex do not exist line " << __LINE__ << " in file " << __FILE__;
				throw std::runtime_error( str_test.str() );
			}
			else{
				vfl_maker.InsertEdge(j, i, NULL);
			}

		}
	}

	ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graphvfl_tmp(&vfl_maker);
	graphvfl_tmp.SetNodeDestroyer(new RegionDestroyer());
	// Install the attribute comparator
	// This needs to be done only on graph g1.
	double my_threshold=0.3;
	graphvfl_tmp.SetNodeComparator(new RegionComparatorHeat(my_threshold));

	return graphvfl_tmp;

}






bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > extractRegions(std::vector<std::pair < int, int > > nodes, ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType>& input, ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType>& model){

	bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > hypo_out;
	bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > hyp;
	for(size_t i = 0  ; i < nodes.size() ; ++i){
		bettergraph::MatchComparable<AASS::graphmatch::Region> match(*(input.GetNodeAttr(nodes[i].first)), *(model.GetNodeAttr(nodes[i].second)) );
		hypo_out.push_back(match);
	}

	return hypo_out;
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



bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > matchTest(ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType>& graph_vfl, ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType>& graph_vfl_model){
 
	//Graph subgraph isomorphisme using VF2
	VF2SubState s0_load(&graph_vfl, &graph_vfl_model);

	std::cout << std::endl << std::endl << "Second test " << std::endl;
	std::vector<std::pair < int, int > > vec;
	if(!match(&s0_load, AASS::visitorVector, &vec)){
		std::cout << "No matching found" << std::endl;
	}

	std::cout << "Input graph : smallest " << graph_vfl.NodeCount() << std::endl;
	std::cout << "model graph : biggest " << graph_vfl_model.NodeCount() << std::endl;

	bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > vflhyp;

	bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > hyp;
	for(size_t i = 0  ; i < vec.size() ; ++i){
		bettergraph::MatchComparable<AASS::graphmatch::Region> match(*(graph_vfl.GetNodeAttr(vec[i].first)), *(graph_vfl_model.GetNodeAttr(vec[i].second)) );
		vflhyp.push_back(match);
	}
//	vflhyp.extractPlace(vec, graph_vfl, graph_vfl_model);

	std::cout << "Matching size " << vflhyp.size() << " and vec size " << vec.size() << std::endl;

	return vflhyp;

}



cv::Mat makeGraph(const std::string& file, AASS::RSI::GraphZoneRI& graph_slam){

	cv::Mat slam1 = cv::imread(file, CV_LOAD_IMAGE_GRAYSCALE);
/** Segmenting the map**/
	AASS::maoris::Segmentor segmenteur;
	AASS::maoris::GraphZone graph_segmented;

	double time = 0;
// 	makeGraph(slam, graph_slam, time);
	time = segmenteur.segmentImage(slam1, graph_segmented);
	cv::Mat segmented_map = segmenteur.getSegmentedMap();

	cv::imshow("Segmented", segmented_map);
	cv::waitKey(0);

	graph_slam = AASS::RSI::GraphZoneRI(graph_segmented);

	graph_slam.updatePCA();
	graph_slam.setPCAClassification();
	graph_slam.setSizesClassification();

	return segmented_map;
}



int main(int argc,      // Number of strings in array argv
         char *argv[]   // Array of command-line argument strings
){


	std::cout << "THE REAL THING" << std::endl;

	bool is_sketch = true;
//	int argc = boost::unit_test::framework::master_test_suite().argc;
//	char** argv = boost::unit_test::framework::master_test_suite().argv;

	std::string file;
	if(argc > 1){
		file = argv[1];
	}
	else{
		file = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/01.png";
	}

	std::string file2;
	if(argc > 2){
		file2 = argv[2];
	}
	else{
		file2 = "/home/malcolm/AASS/sketch_algorithms/Test/RSI/model_simple.png";
	}

	if(argc > 3){
		std::stringstream ss(argv[3]);
		if(!(ss >> std::boolalpha >> is_sketch)) {
			std::cout << "NOT CORRECT BOOLEAN VALUE" << std::endl;
		}
	}
	if(is_sketch) std::cout << "Is a SKETCH" << std::endl;
	else std::cout << "Is NOT a sketch" << std::endl;


	AASS::RSI::GraphZoneRI graph_slam;
	AASS::RSI::GraphZoneRI graph_slam_model;
	cv::Mat mat_in = makeGraph(file, graph_slam);
	cv::Mat model = makeGraph(file2, graph_slam_model);

	cv::imshow("map_in", mat_in);
	cv::imshow("model", model);

	cv::waitKey(0);

	/********** Uniqueness *******************************************/
	graph_slam.setSDAwayFromMeanForUniqueness(1);
	graph_slam_model.setSDAwayFromMeanForUniqueness(1);

	/********** Uniqueness *******************************************/
	graph_slam_model.updateUnique();
	if(is_sketch) {
		graph_slam.updateUnique();
	}
	else{
		std::cout << "IS NOT A SKETCH" << std::endl;
		graph_slam.updateUnique(graph_slam_model);
	}

	assert(graph_slam.zoneUniquenessWasCalculated() == true);
	assert(graph_slam_model.zoneUniquenessWasCalculated() == true);


	/********** Hungarian matching of graph onto itself***************/

	std::cout << "Hungarian Match" << std::endl;
	AASS::RSI::HungarianMatcher hungmatch;
	std::vector<int> scores;
	auto match = hungmatch.match(graph_slam, graph_slam_model, scores);

// 	exit(0);

	std::sort(match.begin(), match.end(), [&graph_slam, &graph_slam_model](AASS::RSI::ZoneCompared &match, AASS::RSI::ZoneCompared &match1){
		return match.getSimilarity() < match1.getSimilarity();
	} );


	AASS::graphmatch::GraphLaplacian gl_in;
	AASS::graphmatch::RSIGraphConverter converter;
	converter.graphZonetoGraphLaplacian(graph_slam, gl_in);


	AASS::graphmatch::GraphLaplacian gl_model;
	AASS::graphmatch::RSIGraphConverter converter2;
	converter2.graphZonetoGraphLaplacian(graph_slam_model, gl_model);


	std::deque < AASS::graphmatch::Match > anchors;
	std::deque < AASS::graphmatch::MatchLaplacian > anchors_laplacian;

	for(size_t i = 0 ; i < match.size() ; ++i){

		auto vertex_anchor1 = match[i].source;
		auto vertex_anchor2 = match[i].target;

		//Laplacian
		AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex_region_anchor_source;
		bool foundl = converter.getEquivalentLaplacianRegion(vertex_anchor1, vertex_region_anchor_source);
		AASS::graphmatch::GraphLaplacian::VertexLaplacian vertex_region_anchor_target;
		bool found2l = converter2.getEquivalentLaplacianRegion(vertex_anchor2, vertex_region_anchor_target);

		assert(foundl == true);
		assert(found2l == true);

		std::cout << "Similarity of match added to anchors : " << match[i].getSimilarity() << std::endl;

		AASS::graphmatch::MatchLaplacian match_p_l(vertex_region_anchor_source, vertex_region_anchor_target);
		anchors_laplacian.push_back(match_p_l);

	}

	/************************************************************
	 * ANCHOR MATCHING
	 */

	for(auto anchor : anchors_laplacian) {
		gl_in.addAnchor(anchor.getFirst());
		gl_model.addAnchor(anchor.getSecond());
	}

	gl_in.eigenLaplacian();
	gl_model.eigenLaplacian();

	double time = 5;
	gl_model.propagateHeatKernel(time);
	gl_in.propagateHeatKernel(time);

	/****************************************************************************
	 * Conversion
	 *
	 * **************************************************************************/


	ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graph_vfl = graphLaplacian2VFL(gl_in);
	ARGraph<AASS::graphmatch::Region, AASS::graphmatch::EdgeType> graph_vfl_model = graphLaplacian2VFL(gl_model);

	std::cout << "Size of input " << graph_slam.getNumVertices() << std::endl;
	std::cout << "Size of model " << graph_slam_model.getNumVertices() << std::endl;


	/****************************************************************************
	 * Matching
	 *
	 * **************************************************************************/

	//Smallest graph needs to be first
	if(graph_vfl.NodeCount() < graph_vfl_model.NodeCount()){
		bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > hyp_vfl = matchTest(graph_vfl, graph_vfl_model);
		drawHypo(hyp_vfl, gl_in, gl_model, mat_in, model, "VFL", false, 1);
	}
	else{
		bettergraph::HypotheseBase<bettergraph::MatchComparable<AASS::graphmatch::Region> > hyp_vfl = matchTest(graph_vfl_model, graph_vfl);
		drawHypo(hyp_vfl, gl_in, gl_model, mat_in, model, "VFL", true, 1);
	}

//	AASS::graphmatch::Hypothese hyp_mine = matchTest(graph_slam, graph_slam_model);
//	hyp_mine.drawMoved(graph_slam, graph_slam_model, mat_in, model, "Custom", 1);

	cv::waitKey(0);

	return 0;
}