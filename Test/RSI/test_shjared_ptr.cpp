
#include <stdio.h>
#include <memory>

int main(){
	
	int* kp = new int();
	std::shared_ptr<int> kp_tmp(kp);
	std::shared_ptr<int> kpp = kp_tmp;
	
	
	int* kp1 = new int();
	std::shared_ptr<int> kp_tmp1(kp1);
	std::shared_ptr<int> kpp2 = kp_tmp1;
	
}