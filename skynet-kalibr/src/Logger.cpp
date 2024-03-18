#include "Logger.h"

void logFrame(cv::Mat img, double timestamp, std::string logDir){
	int ns = (int) (1e3 * timestamp);
	std::string filename = logDir + 	std::stoi(ns) + ".png";
	cv::imwrite(filename, img);
}

int main(){
	return 0;
}
