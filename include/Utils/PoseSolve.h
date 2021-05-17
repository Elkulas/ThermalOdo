// C++ 标准库
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>

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
using namespace cv;


void GetPose(vector<cv::Point2f>& vPts1, vector<cv::Point2f>& vPts2, 
            vector<uchar>& vStatus, cv::Mat& R21, cv::Mat& t21, cv::Mat K, int MaxIter);

void FindFundamental(vector<cv::Point2f>& vPts1, vector<cv::Point2f>& vPts2, 
                    vector<bool> &vbMatchesInliers,  vector<vector<size_t> >& vSets, 
                    float &score, cv::Mat &F21, int succ_count, int MaxIter);

void Normalize(const vector<cv::Point2f> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma, int succ_count, 
                        vector<cv::Point2f>& vPts1, vector<cv::Point2f>& vPts2);
