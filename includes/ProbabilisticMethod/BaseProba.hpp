#ifndef PROBABILITIES_BASETABLE_MAP
#define PROBABILITIES_BASETABLE_MAP

#include <boost/math/distributions/normal.hpp>

namespace AASS{
	namespace probabilisticmatching{
		
		/**
		 * @brief table class to store and calculate edge related probabilities P[F,F| U = U].
		 */
		class BaseProba {

		public:
			BaseProba(){};
			virtual  ~BaseProba(){};
			
			/**
			* @brief Calcule the probability based on the normal distribution.
			* Since it represent the pdf, the probability is given by the integral between two values. Must use the cdf
			*/
			virtual double normalDistribution(double variance_model_1, double e2_input, double e1_input, int e_model);
			
			/**
			* @brief Calcule the probability based on the normal distribution using the standard deviation as an interval.
			* Since it represent the pdf, the probability is given by the integral between two values. Must use the cdf
			*/
			virtual double normalDistribution(double variance_model_1, double e1_input, int e_model);
			virtual double binomialDistribution();
		};
		

		
		
		inline double BaseProba::normalDistribution(double variance_model_1, double e2_input, double e1_input, int e_model)
		{

		// 	boost::math::normal _nd(mean, standard deviation); 
			//standard deviation is square root of variance.
			double sdeviation = sqrt(variance_model_1);
// 			std::cout << "std deviation " << sdeviation << " variance of model " << variance_model_1 << " edge number mkodel" << e_model <<" edge number" << e1_input << std::endl;
			
			boost::math::normal nd(e_model, sdeviation);
			//pdf stands for probibility density function
			//The probability density function is nonnegative everywhere, and its integral over the entire space is equal to one
			double var =  boost::math::cdf(nd, e1_input);
			double var2 =  boost::math::cdf(nd, e2_input);
			double res = 0 ;
			if(var < var2){
				res = var2 - var;
			}
			else{
				res = var - var2;
			}
			if(res > 1 || res < 0){
				throw std::runtime_error("the cdf in base proba failed and it's not supposed to EVER");
			}
			return res;
		}
		
		/**
		 * @brief Calcule the probability based on the normal distribution.
		 * Since it represent the pdf, the probability is given by the integral between two values. Must use the cdf
		 */
		inline double BaseProba::normalDistribution(double variance_model_1, double e1_input, int e_model)
		{

		// 	boost::math::normal _nd(mean, standard deviation); 
			//standard deviation is square root of variance.
			double sdeviation = sqrt(variance_model_1);
// 			std::cout << "std deviation " << sdeviation << " variance of model " << variance_model_1 << " edge number mkodel" << e_model <<" edge number" << e1_input << std::endl;
			
			boost::math::normal nd(e_model, sdeviation);
			//pdf stands for probibility density function
			//The probability density function is nonnegative everywhere, and its integral over the entire space is equal to one
			double var =  boost::math::cdf(nd, e1_input + sdeviation);
			double var2 =  boost::math::cdf(nd, e1_input - sdeviation);
			double res = var - var2 ;
			if(res > 1 || res < 0){
				throw std::runtime_error("the cdf in base proba failed and it's not supposed to EVER");
			}
			return res;
		}
		
		inline double BaseProba::binomialDistribution()
		{
			
// 			boost::math::binomial_distribution<double>();
			return 0;
		}
		
		
	}
}
#endif