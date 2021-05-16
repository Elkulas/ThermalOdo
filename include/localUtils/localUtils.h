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

#include "projection.h"

using namespace std;

/**
 * @brief 生成等距矩阵T,输入为Mat
 * @param[in]       R       rotation
 * @param[in]       t       translation
 * @param[out]      T       等距变换矩阵
 */
Eigen::Isometry3d GenerateT(cv::Mat& R, cv::Mat& t);

/**
 * @brief 将三维关键点根据T来投影到另一个图像平面上
 * @param[in]       keypoints       三维的关键点
 * @param[in]       rgb             投影图像
 * @param[in]       T21             三维关键点到图像的变换关系
 * @param[in]       K               图像内参
 */
void MapFrameToFrame(vector<Eigen::Vector3d>& keypoints, cv::Mat& rgb, Eigen::Isometry3d& T21, Eigen::Matrix3d& K);

void GeneratePointCloud(cv::Mat& color1, cv::Mat& depth1, cv::Mat& color2, cv::Mat& depth2, Eigen::Isometry3d& T1, Eigen::Isometry3d& T2);

void GeneratePointCloudwithK(cv::Mat& color1, cv::Mat& depth1, cv::Mat& color2, cv::Mat& depth2, Eigen::Isometry3d& T1, Eigen::Isometry3d& T2, Eigen::Matrix3d& K);

