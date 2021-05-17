
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <stdio.h>

#include <fstream>
#include <condition_variable>

#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV 库
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;

void GetImageData(vector<pair<double, cv::Mat>>& imgDataset, string& sData_path, string& sConfig_path);

void GetImageDataWithNum(vector<pair<double, cv::Mat>>& imgDataset, string& sData_path, string& sConfig_path, int num);




