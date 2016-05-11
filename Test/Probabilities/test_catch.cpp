#include <iostream>
#include <stdio.h>



int main(){

// 	try{
				
		//pdf stands for probibility density function
		//The probability density function is nonnegative everywhere, and its integral over the entire space is equal to one
		throw std::runtime_error("the pdf failed and it's not supposed to EVER");
	/*}*//*catch(const std::exception &e){
		
		std::cout << "the unthinkable happened : " << e.what() << std::endl;
	}	*/
	
	std::cout << "is this printed ?" << std::endl;
	return 0;
	
}