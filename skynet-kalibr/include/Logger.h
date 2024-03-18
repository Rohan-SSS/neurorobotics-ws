#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <vector>
#include <string>


void logFrame(cv::Mat img, double timestamp);
