#ifndef SKETCHALGORITHM_GRAPHMATCHERBASETEMPLATE_02112018
#define SKETCHALGORITHM_GRAPHMATCHERBASETEMPLATE_02112018

#include <sys/stat.h>

#include "GraphLaplacian.hpp"
#include "HypotheseLaplacian.hpp"
//#include "Cluster.hpp"

//TODO : move the conception of graphPlace in GraphMatcherBase if possible so that its all done internally and graphplace is private. If shared, we can have a double free.
namespace AASS{
	namespace graphmatch{

		/**
		 * @brief Base class of all Graph Matching Classes.
		 */
		class GraphMatcherBaseLaplacian{

		protected :


			std::deque<graphmatch::HypotheseLaplacian> _hypothesis_final;
			// 		graphmatch::Hypothese _hypothesis_final_clustered;


			// 	private:
			// 		std::deque<graphmatch::Hypothese> _hypothesis_to_cluster;

		public :
			GraphMatcherBaseLaplacian(){};
			virtual ~GraphMatcherBaseLaplacian(){}

// 			virtual void inputMaps(const SketchMap& m, const SketchMap& mm);
// 			virtual void inputNewMap(const SketchMap& m);
// 			virtual void inputNewModel(const SketchMap& m);

// 			virtual void drawModelMat(cv::Mat& tmp) const {
// 				tmp = _map_model.getObstacleMat().clone();
// 				_graph_place_model.draw(tmp);
// 				}
// 			virtual SketchMap& getMapModel(){return _map_model;}
// 			virtual const SketchMap& getMapModel() const {return _map_model;}
// 			virtual void drawMat(cv::Mat& tmp) const {
// 				tmp = _map_one.getObstacleMat().clone();
// 				_graph_place.draw(tmp);
// 				}
// 			virtual SketchMap& getMap(){return _map_one;}
// 			virtual const SketchMap& getMap() const {return _map_one;}

			virtual const std::deque<graphmatch::HypotheseLaplacian>& getResult() const {return _hypothesis_final;};

// 			virtual void thinkALL(){
//
// 				_map_model.think();
// 				_map_one.think();
// 				_graph_list_model = _map_model.getGraphLines();
// 				_graph_list = _map_one.getGraphLines();
// 				_graph_place_model = _map_model.getGraphPlace();
// 				_graph_place = _map_one.getGraphPlace();
//
//
// 			}
//
// 			virtual void changeModeGraphFusion(){
// 				_map_model.changeModeGraphFusion();
// 				_map_one.changeModeGraphFusion();
// 			}
//

			//Map matching

			/**
			* @brief Sort the deque from the smallest edit distance to the biggest.
			*
			* The function sort the deque from the smallest edit distance to the biggest. Small = good = not many change to match the graphs.
			*
			* @param[in] hypothesis : deque of Hypothese to be sorted
			*/
			virtual void sort(std::deque< graphmatch::HypotheseLaplacian >& hypothesis);

			/**
			* @brief Sort the deque from the biggest hypothesis to the smallest one, considering the number of _num_vertex.
			*
			**/
			virtual void sort_by_size(std::deque< graphmatch::HypotheseLaplacian >& hypothesis);

			/**
			* @brief reset all attributes
			*/
			virtual void reset();

			/**
			* @brief match vertices by type. Only consider room or not for now.
			*/



			/**
			* @brief : export the result as a drawing
			*
			* @param[in] gp_real : graphmatch::GraphLaplacian input
			* @param[in] gp_model : graphmatch::GraphLaplacian model
			* @param[in] list_result : deque of Match
			* @param[in] name : image name
			*/
			void drawExportData(graphmatch::GraphLaplacian& gp_real, graphmatch::GraphLaplacian& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::deque< graphmatch::Match>& list_result, const std::string& name);



			/**
			* @brief Matching two graphs
			*/
			virtual bool match(graphmatch::GraphLaplacian& gp, graphmatch::GraphLaplacian& gp2) = 0;

			// 		virtual void draw();
			virtual void clear(){_hypothesis_final.clear();}

		};






		inline void AASS::graphmatch::GraphMatcherBaseLaplacian::drawExportData(graphmatch::GraphLaplacian& gp_real, graphmatch::GraphLaplacian& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::deque< graphmatch::Match>& list_result, const std::string& name)
		{

			cv::Mat obst_copy;
			obstacle.copyTo(obst_copy);

			cv::Mat obst_model_copy;
			obstacle_model.copyTo(obst_model_copy);

			cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);


			int cols_max = obst_model_copy.size().width;
			if(cols_max < obst_copy.size().width){
				cols_max = obst_copy.size().width;
			}

			cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
			cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);

			cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));

			obst_copy.copyTo(roi_maps);
			obst_model_copy.copyTo(roi_model_maps);

			cv::Scalar color;
			cv::RNG rrng(12345);

			if(draw_links.channels() == 1){
				color = rrng.uniform(50, 255);
			}

			else if(draw_links.channels() == 3){
				color[1] = rrng.uniform(50, 255);
				color[2] = rrng.uniform(50, 255);
				color[3] = rrng.uniform(50, 255);
			}

			cv::Scalar color_model;

			if(draw_links.channels() == 1){
				color_model = 100;
			}

			else if(draw_links.channels() == 3){
				color_model[1] = 100;
				color_model[2] = 100;
				color_model[3] = 100;
			}

			cv::Scalar color_one;

			if(draw_links.channels() == 1){
				color_one = 255;
			}

			else if(draw_links.channels() == 3){
				color_one[1] = 255;
				color_one[2] = 255;
				color_one[3] = 255;
			}


			for(size_t i = 0 ; i < list_result.size() ; i++ ){
				// 		std::cout << "this : ";
				// 		gp_real.print(list_result[i].getFirst());
				// 		std::cout << " linked to this : " ;
				// 		gp_model.print(list_result[i].getSecond());
				// 		std::cout << std::endl;


				cv::Point2i model;
				model = gp_model[list_result[i].getSecond()].getCenter();
				model.y = model.y + obst_copy.size().height;

				cv::Point2i model_normal;
				model_normal = gp_real[list_result[i].getFirst()].getCenter();


				model_normal.x = model_normal.x * 2 ;
				model_normal.y = model_normal.y * 2 ;
				model = gp_model[list_result[i].getSecond()].getCenter();
				model.x = model.x * 2 ;
				model.y = model.y * 2 ;
				model.y = model.y + obst_copy.size().height;

				cv::line(all_maps, model_normal, model, color, 5);
				cv::circle(all_maps, model_normal, 10, color_model, 3);

				cv::line(all_maps, model_normal, model, color, 5);
				cv::circle(all_maps, model, 10, color_model, 3);


			}
			// 	cv::imshow(name + " maps", all_maps);
			mkdir("ExportGraphMatcherLaplacian", S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
			cv::imwrite("ExportGraphMatcherLaplacian/"+name+".png", all_maps);

		}







	}
}

#endif