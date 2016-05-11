#ifndef TIMER_UTILITY_SKETCH_MAP
#define TIMER_UTILITY_SKETCH_MAP

#include <ctime> 
#include <boost/format.hpp>


//To call the function with argument boost::bind
inline double timed( 
	const std::string& name,
	const boost::function< void()> &callback
){

	//Launch the call back function in between the timers
	std::clock_t a = std::clock();
	callback();
	std::clock_t b = std::clock();	
	
	std::cout << boost::format("CPU time %-5i in second %-10i for %-10in") % (b - a) % ( ((float)(b - a))/CLOCKS_PER_SEC) % name << std::endl;
	
// 	std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< " for " << name << std::endl;
	
	return ( ((float)(b - a))/CLOCKS_PER_SEC);
}

// template< typename T>
// inline T timedwe( 
// 	const std::string& name,
// 	const boost::function< T()> &callback
// ){
// 
// 	//Launch the call back function in between the timers
// 	std::clock_t a = std::clock();
// 	T res = callback();
// 	std::clock_t b = std::clock();	
// 	
// 	std::cout << "CPU time " << (b - a) << " or in seconds " <<( ((float)(b - a))/CLOCKS_PER_SEC)<< " for " << name << std::endl;
// 	
// 	return res;
// }

#endif