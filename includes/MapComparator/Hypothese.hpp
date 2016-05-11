#ifndef COMPARATOR_HYPOTHESE_MAP
#define COMPARATOR_HYPOTHESE_MAP

#include "Match.hpp"
#include "StorageFilter.hpp"
#include "GraphPlace.hpp"
#include "boost/bind.hpp"

namespace AASS{
	namespace graphmatch{	
		
		/**
		 * @brief Object defining a possible matching between two graphmatch::GraphPlace
		 *
		 * As a deque of Match, an Hypothese define a possible matching between an input graph and a model graph 
		 * 
		 */
		class Hypothese{
			
		protected :
			
			std::deque< graphmatch::Match > _hypothesis;
			int _dist;
			
	// 		typedef std::deque< std::pair < topologicalmap::VertexPlace, topologicalmap::VertexPlace > >::iterator hypo_iterator ;
				
		public :
			
			Hypothese() : _dist(0){};
			Hypothese(const std::deque< graphmatch::Match >& h, int d) : _hypothesis(h), _dist(d){};
			
			void setDist(int d){_dist = d;}
			int getDist(){return _dist;}
			int getDist() const {return _dist;}
			size_t size() const {return _hypothesis.size();}
			void push_back(const graphmatch::Match& pair){_hypothesis.push_back(pair);}
			void push_front(const graphmatch::Match& pair){_hypothesis.push_front(pair);}
			void pop_back(){_hypothesis.pop_back();}
			void pop_front(){_hypothesis.pop_front();}
			void clear(){_hypothesis.clear(); _dist = 0 ;}
			bool empty(){return _hypothesis.empty();}
			graphmatch::Match& at(int i){return _hypothesis.at(i);}
			std::deque< graphmatch::Match>::iterator begin(){return _hypothesis.begin();}
			std::deque< graphmatch::Match >::iterator erase(const std::deque< graphmatch::Match>::iterator& it){return _hypothesis.erase(it);};
			
			
			std::deque< graphmatch::Match >& getMatches(){return _hypothesis;}
			const std::deque< graphmatch::Match >& getMatches() const {return _hypothesis;}
			
			//TODO : in isSeen no need for the double comparison because it's supposed to be consistent.
			
			/**
			 * @brief Tell if the input match is part of the Hypothese
			 * 
			 * @param[in] pair : graphmatch::Match that we want to test
			 * @return true if seen, false otherwise
			 * 
			 */
			bool isSeen(const graphmatch::Match& pair) const;
			
			/**
			 * @brief Tell if the input match is part of the Hypothese and return the index
			 * 
			 * @param[in] pair : graphmatch::Match that we want to test
			 * @param[in] index : index of the Match in Hypothese
			 * @return true if seen, false otherwise
			 */
			bool isSeen(const graphmatch::Match& pair, int& index) const;
			
			/**
			 * @brief Tell if the input match is part of the Hypothese and return the index
			 * 
			 * @param[in] pair : graphmatch::Match that we want to test
			 * @param[in] index : index of the Match in Hypothese
			 * @return true if seen, false otherwise
			 */
			bool isSeen(const graphmatch::VertexPlace& vp, int& index) const;
			
			/**
			 * @brief Test if two hypothese don't have overlaping points.
			 * 
			 * @param[in] h : Hypothese to test
			 * @return true if compatible, false otherwise. 
			 */
			bool isCompatible(const graphmatch::Hypothese h) const;
			
			graphmatch::Match& operator[](int i);
			const graphmatch::Match& operator[](const int i) const;
			
			/**
			 * @brief Fuse two Hypthese
			 * 
			 * @param[in] hyp : Hypothese to fuse to *this*
			 */
			void fuse(const graphmatch::Hypothese& hyp);
			
			/**
			 * @brief Fuse  a deque of Hypothese to *this*
			 * 
			 * @param[in] clu : deque of Hypothese to fuse to *this*
			 */
			void fuse(const std::deque<graphmatch::Hypothese>& clu);
			
			/**
			 * @brief calculate the edit distance of two graph depending on *this* Hypothese
			 * 
			 * Every vertex added or removed = 1
			 * 
			 * Every edge added or removed = 1
			 * 
			 * @param[in] gp : graphmatch::GraphPlace input
			 * @param[in] gp_model : graphmatch::GraphPlace model 
			 */
			int updateDistance(AASS::graphmatch::GraphPlace& gp, AASS::graphmatch::GraphPlace& gp_model) const;
			
			//TODO : Make sure it works still after multiple iteration. It might be broken with a cost of zero anyway.
			
			/**
			 * @brief get the size of the zone with vertex of same value or less
			 * 
			 * @param[in] vertex : vertex you want to have the zone size for
			 * @return size of the zone
			 * 
			 */
			int getSizeSimilarZone(AASS::graphmatch::GraphPlace& gp, const AASS::graphmatch::VertexPlace& vertex);
			
			int getSizeSimilarZone(graphmatch::GraphPlace& gp, const AASS::graphmatch::VertexPlace& vertex, Hypothese& zone);
			
			
			/********************************
			 * Drawing functions
			 * *****************************/
			
			/**
			* @brief draw
			* 
			* @param[in] gp_real : graphmatch::GraphPlace of the input
			* @param[in] gp_model : graphmatch::GraphPlace of the model
			* @param[in] _hypothesis : deque of Match between gp_real and gp_model
			* @param[in] name : windows name
			* @param[in] mat_out : drawing out
			*/
			void drawHypo(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, cv::Mat& mat_out) const;
			
			/**
			* @brief draw
			* 
			* @param[in] gp_real : graphmatch::GraphPlace of the input
			* @param[in] gp_model : graphmatch::GraphPlace of the model
			* @param[in] _hypothesis : deque of Match between gp_real and gp_model
			* @param[in] name : windows name
			*/
			void drawHypo(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name) const;
			
			/**
			* @brief scale the graph and draw 
			* 
			* @param[in] gp_real : graphmatch::GraphPlace of the input
			* @param[in] gp_model : graphmatch::GraphPlace of the model
			* @param[in] _hypothesis : deque of Match between gp_real and gp_model
			* @param[in] name : windows name
			* @param[in] scale : scale to resize the graph.
			*/
			void drawHypo(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale, cv::Mat& mat_out) const;
			
			/**
			* @brief scale the graph and draw 
			* 
			* @param[in] gp_real : graphmatch::GraphPlace of the input
			* @param[in] gp_model : graphmatch::GraphPlace of the model
			* @param[in] _hypothesis : deque of Match between gp_real and gp_model
			* @param[in] name : windows name
			* @param[in] scale : scale to resize the graph.
			*/
			void drawHypo(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const;
			
			/**
			* @brief draw one match by one
			* 
			* @param[in] gp_real : graphmatch::GraphPlace of the input
			* @param[in] gp_model : graphmatch::GraphPlace of the model
			* @param[in] _hypothesis : deque of Match between gp_real and gp_model
			* @param[in] name : windows name
			*/
			void drawHypoSlow(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name) const;
			
			/**
			* @brief draw one match by one
			* 
			* @param[in] gp_real : graphmatch::GraphPlace of the input
			* @param[in] gp_model : graphmatch::GraphPlace of the model
			* @param[in] _hypothesis : deque of Match between gp_real and gp_model
			* @param[in] name : windows name
			* @param[in] scale : scale to resize the graph.
			*/
			void drawHypoSlow(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const;
			
			/**
			* @brief init the graphmatch::GraphPlace the graphmatch is going ot be working on
			* 
			* @param[in] gp : graphmatch::GraphPlace input
			* @param[in] gp2 : graphmatch::GraphPlace model
			*/
// 			virtual void init(const graphmatch::GraphPlace& gp, const graphmatch::GraphPlace& gp2);

			void drawMoved(const GraphPlace& gp_real_const, const GraphPlace& gp_model_const, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, int scale) const;
			
			void drawLinks(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale, cv::Mat& out) const;
			
			void drawLinks(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const;
			
			void drawPartialGraphs(graphmatch::GraphPlace& gp_real, graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale, bool export_flag) const;
			
			
		};
		
		inline bool Hypothese::isSeen(const graphmatch::Match& pair) const
		{
			
			for(size_t i = 0 ; i < _hypothesis.size() ; i++){
				if(_hypothesis[i].getFirst() == pair.getFirst() || _hypothesis[i].getSecond() == pair.getSecond()){
					return true;
				}
				else if(_hypothesis[i].getSecond() == pair.getFirst() || _hypothesis[i].getFirst() == pair.getSecond()){
					return true;
				}
			}
			return false;
		}
		
		inline bool Hypothese::isSeen(const graphmatch::Match& pair, int& index) const
		{
			
			for(size_t i = 0 ; i < _hypothesis.size() ; i++){
				if(_hypothesis[i].getFirst() == pair.getFirst() && _hypothesis[i].getSecond() == pair.getSecond()){
					index = i ;
					return true;
				}
				else if(_hypothesis[i].getSecond() == pair.getFirst() && _hypothesis[i].getFirst() == pair.getSecond()){
					index = i;
					return true;
				}
			}
			index = -1;
			return false;
		}
		
		
		inline bool Hypothese::isSeen(const graphmatch::VertexPlace& vp, int& index) const
		{
			for(size_t i = 0 ; i < _hypothesis.size() ; i++){
				if(_hypothesis[i].getFirst() == vp || _hypothesis[i].getSecond() == vp){
					index = i ;
					return true;
				}
			}
			index = -1;
			return false;
			
		}
		
		inline graphmatch::Match& Hypothese::operator[](int i)
		{
			return _hypothesis[i];
		}
		inline const graphmatch::Match& Hypothese::operator[](const int i) const
		{
			return _hypothesis[i];
		}
		
		inline bool Hypothese::isCompatible(const Hypothese h) const
		{
			
			bool compatible = true;
			std::deque< graphmatch::Match > match_h;
			match_h = h.getMatches();
			
			std::deque< graphmatch::Match >::iterator it;
			
			for(it = match_h.begin() ; it != match_h.end() ; it++){
				if(isSeen(*it) == true){
					compatible = false;
				}
			}
			
			return compatible;
			

		}
		
		inline void Hypothese::fuse(const Hypothese& hyp)
		{
			std::deque<graphmatch::Match>::const_iterator it;
			for(it = hyp.getMatches().begin() ; it != hyp.getMatches().end() ; it++){
			
				_hypothesis.push_back(*it);
			}
			_dist = _dist + hyp.getDist();
		}

		
		inline void Hypothese::fuse(const std::deque<graphmatch::Hypothese>& clu)
		{
			std::deque<graphmatch::Hypothese>::const_iterator it;
			for(it = clu.begin() ; it != clu.end() ; it++ ){
				fuse(*it);
			}

		}
		
		inline int Hypothese::getSizeSimilarZone(graphmatch::GraphPlace& gp, const AASS::graphmatch::VertexPlace& vertex, Hypothese& zone){
			
			/* Do a partial A* */
			std::deque < VertexPlace> queue;
			
			gp.resetLabel();
			
			queue.push_back(vertex);
			int count = 1;
			
			int index_vertex = 0;
			isSeen(vertex, index_vertex);
			
			double cost_vertex = _hypothesis[index_vertex].getCost();
			zone.push_back(_hypothesis[index_vertex]);
			
			gp[vertex].label = true;
			
			if(index_vertex == -1){
				throw std::runtime_error("Vertex is not part of the hypothese. Error in Hypothese::getSizeSimilarZone");
			}
			
			while(queue.size() != 0){
				
// 				std::cout << "Queue " << queue.size() << std::endl;
				
				//Get all linked vertex
				std::deque < VertexPlace> tmp;
				gp.getAllVertexLinked(queue[0], tmp);
				
				int index = 0 ;
				//If not seen before AND part of the hypothese in model AND input and cost is the same of less, add to list augment the count by one
				for(size_t i = 0 ; i < tmp.size() ; ++i){
					if(gp[tmp[i]].label == false && isSeen(tmp[i], index) == true){
						if(cost_vertex >= _hypothesis[index].getCost() ){
// 							std::cout << "Good index " << index << " csts " << cost_vertex << " " << _hypothesis[index].getCost() << std::endl;
							gp[tmp[i]].label = true;
							queue.push_back(tmp[i]);
							
							++count;
							zone.push_back(_hypothesis[index]);
							
						}
					}
				}
				
				queue.pop_front();
				
			}
			
			return count;
			
			
		}
		
		
		inline int Hypothese::getSizeSimilarZone(graphmatch::GraphPlace& gp, const AASS::graphmatch::VertexPlace& vertex)
		{
			
			Hypothese hyp;
			return getSizeSimilarZone(gp, vertex, hyp);
			
		}
		
		
		/*********************************************************************
		 * Drawing functions
		 * *******************************************************************/
		
		inline void AASS::graphmatch::Hypothese::drawHypo(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale, cv::Mat& mat_out) const 
		{
			
			cv::Mat obst_copy;
			obstacle.copyTo(obst_copy);
			
			cv::Mat obst_model_copy;
			obstacle_model.copyTo(obst_model_copy);
			
			cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
			cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
			cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
			
			int cols_max = obst_model_copy.size().width;
			if(cols_max < obst_copy.size().width){
				cols_max = obst_copy.size().width;
			}
			
			cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
			cv::Mat all = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);
			
			cv::Mat roi = all(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_linked = only_linked(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model = all(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			obst_copy.copyTo(roi_maps);
			obst_model_copy.copyTo(roi_model_maps);
			
			cv::Scalar color;
				
			if(draw_links.channels() == 1){
				color = 255;

			}
			
			else if(draw_links.channels() == 3){
				color[0] = 500;
				color[1] = 500;
				color[2] = 500;
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
			
			cv::Scalar color_link_maps;
				
			if(all_maps.channels() == 1){
				color_link_maps = 255;
			}
			
			else if(all_maps.channels() == 3){
				color_link_maps[0] = 255;
				color_link_maps[1] = 255;
				color_link_maps[2] = 255;
			}
			
			//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
			gp_real.scale(scale);
			gp_model.scale(scale);
			
			gp_real.drawSpecial(roi);
			gp_model.drawSpecial(roi_model);
			
			for(size_t i = 0 ; i < _hypothesis.size() ; i++ ){
				
				cv::line(draw_links, gp_real[_hypothesis[i].getFirst()].mass_center, gp_model[_hypothesis[i].getSecond()].mass_center, color);
				
				
				//UGLY HACK BECAUSE THE SCALING DOESN'T WORK 
				cv::Point2i model_normal;
				model_normal = gp_real[_hypothesis[i].getFirst()].mass_center;
						
				cv::Point2i model = gp_model[_hypothesis[i].getSecond()].mass_center;
				model.y = model.y + obst_copy.size().height;
				
				cv::circle(draw_links, model, 10, color_model, 3);
				cv::circle(draw_links, model_normal, 10, color_one, 3);
				
				cv::line(all, model_normal, model, color, 5);
				
				cv::Scalar color_all_linked;
				cv::RNG rrng(12345);
				if(all.channels() == 1){
					color_all_linked = rrng.uniform(50, 255);
				}
				
				else if(all.channels() == 3){
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
			
			
			gp_real.drawSpecial(draw_graph);
			gp_model.drawSpecial(draw_graph_model);
			
			gp_real.scale(1/scale);
			gp_model.scale(1/scale);
			
		// 	cv::imshow(name + "links", draw_links);
// 			cv::imshow("graph place", draw_graph);
// 			cv::imshow("model graph place", draw_graph_model);
			
// 			cv::imshow(name + " maps", all_maps);
		// 	cv::imshow(name + " only linked", only_linked);
			
			mat_out = all;
			
		}

		inline void AASS::graphmatch::Hypothese::drawHypo(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, cv::Mat& mat_out) const
		{
			drawHypo(gp_real, gp_model, obstacle, obstacle_model, name, 1, mat_out);
		}
		
		inline void AASS::graphmatch::Hypothese::drawHypo(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name) const
		{
			cv::Mat tmp;
			drawHypo(gp_real, gp_model, obstacle, obstacle_model, name, 1, tmp);
			cv::imshow(name, tmp);
		}
		
		inline void AASS::graphmatch::Hypothese::drawHypo(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const 
		{
			cv::Mat tmp;
			drawHypo(gp_real, gp_model, obstacle, obstacle_model, name, scale, tmp);
			cv::imshow(name, tmp);
		}
		
		

		inline void AASS::graphmatch::Hypothese::drawHypoSlow(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name) const
		{
			drawHypoSlow(gp_real, gp_model, obstacle, obstacle_model, name, 1);
		}


		inline void AASS::graphmatch::Hypothese::drawHypoSlow(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const 
		{
			
			cv::Mat obst_copy;
			obstacle.copyTo(obst_copy);
			
			cv::Mat obst_model_copy;
			obstacle_model.copyTo(obst_model_copy);
			
			cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
			cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
			cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
			
			int cols_max = obst_model_copy.size().width;
			if(cols_max < obst_copy.size().width){
				cols_max = obst_copy.size().width;
			}
			
			cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
			cv::Mat all = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);
			
			cv::Mat roi = all(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_linked = only_linked(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model = all(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			gp_real.drawSpecial(roi);
			gp_model.drawSpecial(roi_model);
			
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
			
			
			for(size_t i = 0 ; i < _hypothesis.size() ; i++ ){
		// 		std::cout << "this : ";
		// 		gp_real.print(_hypothesis[i].getFirst());
		// 		std::cout << " linked to this : " ;
		// 		gp_model.print(_hypothesis[i].getSecond());
		// 		std::cout << std::endl;
				
				cv::line(draw_links, gp_real[_hypothesis[i].getFirst()].mass_center, gp_model[_hypothesis[i].getSecond()].mass_center, color);
				
				cv::Point2i model;
				model = gp_model[_hypothesis[i].getSecond()].mass_center;
				model.y = model.y + obst_copy.size().height;
				
				cv::Point2i model_normal;
				model_normal = gp_real[_hypothesis[i].getFirst()].mass_center;
				
				cv::line(all, model_normal, model, color);
				
				cv::circle(draw_links, model, 10, color_model, 3);
				cv::circle(draw_links, model_normal, 10, color_one, 3);
						
				model_normal.x = model_normal.x * 2 ;
				model_normal.y = model_normal.y * 2 ;
				model = gp_model[_hypothesis[i].getSecond()].mass_center;
				model.x = model.x * 2 ;
				model.y = model.y * 2 ;
				model.y = model.y + obst_copy.size().height;
				
		// 		cv::line(all_maps, model_normal, model, color);
				cv::line(only_linked, model_normal, model, color);
				cv::circle(only_linked, model_normal, 10, color_one, 3);
				cv::line(all_maps, model_normal, model, color);
				cv::circle(all_maps, model_normal, 10, color_model, 3);
			
				cv::circle(only_linked, model, 10, color_model, 3);
				cv::line(all_maps, model_normal, model, color);
				cv::circle(all_maps, model, 10, color_model, 3);
			
				cv::imshow(name + "links", draw_links);
				cv::imshow("graph place", draw_graph);
				cv::imshow("model graph place", draw_graph_model);
				cv::imshow(name, all);
				cv::imshow(name + " maps", all_maps);
				cv::imshow(name + " only linked", only_linked);
				cv::waitKey(0);
				
				
				only_linked.setTo(cv::Scalar(0, 0, 0));
				roi.setTo(cv::Scalar(0, 0, 0));
				roi_model.setTo(cv::Scalar(0, 0, 0));
				
				
				gp_real.drawSpecial(roi);
				gp_model.drawSpecial(roi_model);
				
				
				
			}
			
			gp_real.scale(scale);
			gp_model.scale(scale);
			
			gp_real.drawSpecial(draw_graph);
			gp_model.drawSpecial(draw_graph_model);
			
			gp_real.scale(1/scale);
			gp_model.scale(1/scale);
			
			cv::imshow(name + "links", draw_links);
			cv::imshow("graph place", draw_graph);
			cv::imshow("model graph place", draw_graph_model);
			cv::imshow(name, all);
			cv::imshow(name + " maps", all_maps);
			cv::imshow(name + " only linked", only_linked);
			
		}
		
		
		inline void Hypothese::drawMoved(const GraphPlace& gp_real_const, const GraphPlace& gp_model_const, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, int scale) const
		{
			
			GraphPlace gp_real_copy = gp_real_const;
			GraphPlace gp_real = gp_real_const;
			GraphPlace gp_model_copy = gp_model_const;
			
			std::deque< Match > list_result = _hypothesis;
			std::deque< Match > list_result_real = _hypothesis;
			
			//Change the value of the vertex
			for(size_t i = 0 ; i < list_result.size() ; ++i){
				std::pair<VertexIteratorPlace, VertexIteratorPlace> vp;
				for (vp = boost::vertices(gp_real_copy.getGraph()); vp.first != vp.second; ++vp.first) {
					VertexPlace v = *vp.first;
// 					std::cout << "MC " << gp_real_copy[v].mass_center << std::endl;
					if (gp_real_copy[v].mass_center == gp_real_const[ list_result[i].getFirst()].mass_center){
						list_result[i].setFirst(v);
					}
				}
			}
			
			for(size_t i = 0 ; i < list_result.size() ; ++i){
				std::pair<VertexIteratorPlace, VertexIteratorPlace> vp;
				for (vp = boost::vertices(gp_real.getGraph()); vp.first != vp.second; ++vp.first) {
					VertexPlace v = *vp.first;
// 					std::cout << "MC " << gp_real_copy[v].mass_center << std::endl;
					if (gp_real[v].mass_center == gp_real_const[ list_result[i].getFirst()].mass_center){
						list_result_real[i].setFirst(v);
					}
				}
			}
			
			//Change the value of the vertex second
			for(size_t i = 0 ; i < list_result.size() ; ++i){
				std::pair<VertexIteratorPlace, VertexIteratorPlace> vp;
				for (vp = boost::vertices(gp_model_copy.getGraph()); vp.first != vp.second; ++vp.first) {
					VertexPlace v = *vp.first;
// 					std::cout << "MC " << gp_model_copy[v].mass_center << std::endl;
					if (gp_model_copy[v].mass_center == gp_model_const[ list_result[i].getSecond()].mass_center){
						list_result[i].setSecond(v);
					}
				}
			}
			
			for(size_t i = 0 ; i < list_result.size() ; ++i){
// 				std::cout << "num vert " << gp_real_copy.getNumVertices()<< std::endl;
				gp_real_copy[ list_result[i].getFirst() ].mass_center = gp_model_copy[ list_result[i].getSecond() ].mass_center;
			}
// 			std::cout << "num vert " << gp_real_copy.getNumVertices()<< std::endl;
			
			cv::Mat obst_copy;
			obstacle.copyTo(obst_copy);
			
			cv::Mat obst_model_copy;
			obstacle_model.copyTo(obst_model_copy);
			
			cv::Mat draw_links = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
			cv::Mat draw_graph = cv::Mat::zeros(obst_copy.size(), CV_8UC3);
			cv::Mat draw_graph_model = cv::Mat::zeros(obst_model_copy.size(), CV_8UC3);
			
			int cols_max = obst_model_copy.size().width;
			if(cols_max < obst_copy.size().width){
				cols_max = obst_copy.size().width;
			}
			
			cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
			cv::Mat all = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat original_graphs = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat original_graphs_part = cv::Mat::zeros(size, CV_8UC3);
			
			cv::Mat only_linked = cv::Mat::zeros(size, CV_8UC3);
			cv::Mat all_maps = cv::Mat::zeros(size, CV_8UC3);
			
			cv::Mat roi = all(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_origraph = original_graphs(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_origraph_part = original_graphs_part(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_linked = only_linked(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model = all(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			cv::Mat roi_model_origraph = original_graphs(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			cv::Mat roi_model_origraph_part = original_graphs_part(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			obst_copy.copyTo(roi_maps);
			obst_model_copy.copyTo(roi_model_maps);
			
			cv::Scalar color;
				
			if(draw_links.channels() == 1){
				color = 255;

			}
			
			else if(draw_links.channels() == 3){
				color[0] = 500;
				color[1] = 500;
				color[2] = 500;
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
			
			cv::Scalar color_link_maps;
				
			if(all_maps.channels() == 1){
				color_link_maps = 255;
			}
			
			else if(all_maps.channels() == 3){
				color_link_maps[0] = 255;
				color_link_maps[1] = 255;
				color_link_maps[2] = 255;
			}
			
			//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
			gp_real_copy.scale(scale);
			gp_model_copy.scale(scale);
			
			
			//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
			gp_real.scale(scale);
			
			
			
			for(size_t i = 0 ; i < list_result.size() ; i++ ){
				
				cv::line(draw_links, gp_real_copy[list_result[i].getFirst()].mass_center, gp_model_copy[list_result[i].getSecond()].mass_center, color);
				
				
				//UGLY HACK BECAUSE THE SCALING DOESN'T WORK 
				cv::Point2i model_normal;
				model_normal = gp_real_copy[list_result[i].getFirst()].mass_center;
						
				cv::Point2i model = gp_model_copy[list_result[i].getSecond()].mass_center;
				model.y = model.y + obst_copy.size().height;
				
				cv::circle(draw_links, model, 10, color_model, 3);
				cv::circle(draw_links, model_normal, 10, color_one, 3);
				
				
				
				cv::Scalar color_all_linked;
				cv::RNG rrng(12345);
				if(all.channels() == 1){
					color_all_linked = rrng.uniform(50, 255);
				}
				
				else if(all.channels() == 3){
					color_all_linked[1] = rrng.uniform(50, 255);
					color_all_linked[2] = rrng.uniform(50, 255);
					color_all_linked[3] = rrng.uniform(50, 255);
				}
				
				cv::circle(all, model, 10, color_all_linked, 3);
				cv::circle(all, model_normal, 10, color_all_linked, 3);
				cv::line(all, model_normal, model, color, 5);
		
				//cv::line(all_maps, model_normal, model, color);
				cv::line(only_linked, model_normal, model, color);
				
				cv::circle(only_linked, model_normal, 10, color_one, -1);
				cv::line(all_maps, model_normal, model, color_link_maps, 5);
				cv::circle(all_maps, model_normal, 10, color, -1);
			
			
				cv::circle(only_linked, model, 10, color_model, -1);
				cv::line(all_maps, model_normal, model, color_link_maps, 5);
				cv::circle(all_maps, model, 10, color, -1);
				
				
			}
			
			std::deque<VertexPlace> dv;
			std::deque<VertexPlace> dv_model;
			for(size_t i = 0 ; i < list_result.size() ; i++){
				dv.push_back(list_result[i].getFirst() );
				dv_model.push_back(list_result[i].getSecond() );
			}
			
			std::deque<VertexPlace> dv_const;
			for(size_t i = 0 ; i < list_result_real.size() ; i++){
				dv_const.push_back(list_result_real[i].getFirst() );
			}
			
			gp_real_copy.drawSpecial(roi, dv);
			gp_model_copy.drawSpecial(roi_model, dv_model);
			
			gp_real.drawSpecial(roi_origraph);
			gp_model_copy.drawSpecial(roi_model_origraph);
			
			gp_real.drawSpecial(roi_origraph_part, dv_const);
			gp_model_copy.drawSpecial(roi_model_origraph_part, dv_model);
			
			gp_real_copy.drawSpecial(draw_graph);
			gp_model_copy.drawSpecial(draw_graph_model);
			
			gp_real_copy.scale(1/scale);
			gp_model_copy.scale(1/scale);
			
		// 	cv::imshow(name + "links", draw_links);
// 			cv::imshow("graph place", draw_graph);
// 			cv::imshow("model graph place", draw_graph_model);
			cv::imshow(name, all);
			cv::imshow("Original graphs", original_graphs);
			cv::imshow("Original graphs partial view", original_graphs_part);
// 			cv::imshow(name + " maps", all_maps);
		// 	cv::imshow(name + " only linked", only_linked);
			

		}
		
		inline void AASS::graphmatch::Hypothese::drawLinks(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale) const{
			
			cv::Mat tmp;
			drawLinks(gp_real, gp_model, obstacle, obstacle_model, name, scale, tmp);
			cv::imshow(name + "links", tmp);
		}
		
		inline void AASS::graphmatch::Hypothese::drawLinks(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale, cv::Mat& out) const 
		{
			
			cv::Mat obst_copy;
			obstacle.copyTo(obst_copy);
			
			cv::Mat obst_model_copy;
			obstacle_model.copyTo(obst_model_copy);

			int cols_max = obst_model_copy.size().width;
			if(cols_max < obst_copy.size().width){
				cols_max = obst_copy.size().width;
			}
			
			cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
			
			cv::Mat all_maps = cv::Mat::zeros(size, obst_copy.type());
// 			std::cout <<"TYPE IOS : " << type2str(obst_copy.type()) << std::endl;
			
			cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			obst_copy.copyTo(roi_maps);
			obst_model_copy.copyTo(roi_model_maps);
			
			all_maps.convertTo(all_maps, CV_8UC3);
			
			cv::Scalar color;
				
			if(all_maps.channels() == 1){
				color = 150;

			}
			
			else if(all_maps.channels() == 3){
				color[0] = 150;
				color[1] = 150;
				color[2] = 150;
			}
			
			
			
			//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
			gp_real.scale(scale);
			gp_model.scale(scale);
			
			for(size_t i = 0 ; i < _hypothesis.size() ; i++ ){
				
				if(gp_real.getNumEdges(_hypothesis[i].getFirst()) > 1 && gp_model.getNumEdges(_hypothesis[i].getSecond()) > 1){
				
					//UGLY HACK BECAUSE THE SCALING DOESN'T WORK 
					cv::Point2i model_normal;
					model_normal = gp_real[_hypothesis[i].getFirst()].mass_center;
							
					cv::Point2i model = gp_model[_hypothesis[i].getSecond()].mass_center;
					model.y = model.y + obst_copy.size().height;
					
	// 				cv::circle(draw_links, model, 10, color_model, 3);
	// 				cv::circle(draw_links, model_normal, 10, color_one, 3);
					
					cv::line(all_maps, model_normal, model, color, 5);
					
					cv::Scalar color_all_linked;
					cv::RNG rrng(12345);
					if(all_maps.channels() == 1){
						color_all_linked = rrng.uniform(50, 255);
					}
					
					else if(all_maps.channels() == 3){
						color_all_linked[1] = rrng.uniform(50, 255);
						color_all_linked[2] = rrng.uniform(50, 255);
						color_all_linked[3] = rrng.uniform(50, 255);
					}
					
					cv::circle(all_maps, model, 10, color_all_linked, 3);
					cv::circle(all_maps, model_normal, 10, color_all_linked, 3);
				}
				
				
			}
			
			gp_real.scale(1/scale);
			gp_model.scale(1/scale);
			
			out = all_maps;
			
		}
		
		
		inline void AASS::graphmatch::Hypothese::drawPartialGraphs(AASS::graphmatch::GraphPlace& gp_real, AASS::graphmatch::GraphPlace& gp_model, const cv::Mat& obstacle, const cv::Mat& obstacle_model, const std::string& name, double scale, bool export_flag) const 
		{
			
			cv::Mat obst_copy;
			obstacle.copyTo(obst_copy);
			
			cv::Mat obst_model_copy;
			obstacle_model.copyTo(obst_model_copy);

			int cols_max = obst_model_copy.size().width;
			if(cols_max < obst_copy.size().width){
				cols_max = obst_copy.size().width;
			}
			
			cv::Size size(cols_max, obst_model_copy.size().height + obst_copy.size().height);
			
			cv::Mat all_maps = cv::Mat::zeros(size, obst_copy.type());
// 			std::cout <<"TYPE IOS : " << type2str(obst_copy.type()) << std::endl;
			
			cv::Mat roi_maps = all_maps(cv::Rect(0,0,obst_copy.size().width,obst_copy.size().height));
			cv::Mat roi_model_maps = all_maps(cv::Rect(0 ,obst_copy.size().height, obst_model_copy.size().width,obst_model_copy.size().height));
			
			obst_copy.copyTo(roi_maps);
			obst_model_copy.copyTo(roi_model_maps);
			
			all_maps.convertTo(all_maps, CV_8UC3);
			
			cv::Scalar color;
				
			if(all_maps.channels() == 1){
				color = 150;

			}
			
			else if(all_maps.channels() == 3){
				color[0] = 150;
				color[1] = 150;
				color[2] = 150;
			}
			
			
			
			//ATTENTION : DOESN'T WORK ON ALREADY STORED DESCRIPTOR SO NEED FOR HACK
			gp_real.scale(scale);
			gp_model.scale(scale);
			
			gp_real.drawJunction(roi_maps);
			gp_model.drawJunction(roi_model_maps);
			
			gp_real.scale(1/scale);
			gp_model.scale(1/scale);
			
			cv::imshow(name + "partial graph", all_maps);
			
			if(export_flag == true){
				std::string out_name = name+".pgm";
				cv::imwrite(out_name, all_maps);
			}
			
		}
		
		
		
		
		
		
		
		
		/* STD::COUT function*/
		
		inline std::ostream& operator<<(std::ostream& os, const AASS::graphmatch::Hypothese& dt){
			os << "Hypothese" << std::endl;
			for(size_t i = 0 ; i < dt.size() ; ++i){
				os << "element " << i << " " << dt[i] << std::endl;
			}
			return os;
		}
		
		
		
		
		
	}
}

#endif