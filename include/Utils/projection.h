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

/**
 * @brief 将内参转换为内参矩阵Ｋ
 * @param[in]       fx      
 * @param[in]       fy      
 * @param[in]       cx      
 * @param[in]       cy
 * @param[out]      K
 */
Eigen::Matrix3d GenerateK(const double& fx, const double& fy, const double& cx, const double& cy);

/**
 * @brief 转换三维点坐标
 * @param[in]       points1          原始三维点集
 * @param[in|out]   points2          输出三维点集
 * @param[in]       T21              转换关系 1->2 
 */
void PointsTransfer(vector<Eigen::Vector3d>& points1, vector<Eigen::Vector3d>& points2, Eigen::Isometry3d& T21);

/**
 * @brief 单点相机投影函数,2D->3D
 * @param[in]       pixel   二维图像坐标
 * @param[in]       K       相机内参矩阵
 * @param[in]       depth   该点的深度
 * @param[in|out]   point   三维点坐标
 */
Eigen::Vector3d Projection(Eigen::Vector3d& pixel, const double& depth, Eigen::Matrix3d& K);

/**
 * @brief 多点相机投影函数,2D->3D, 使用深度图
 * @param[in]       pixels   二维图像坐标集合
 * @param[in]       K        相机内参矩阵
 * @param[in]       depth    对应的深度图
 * @param[in|out]   point    三维点坐标
 */
vector<Eigen::Vector3d> BuckProjection(vector<Eigen::Vector3d>& pixels, cv::Mat& depth, Eigen::Matrix3d& K);

/**
 * @brief 单点相机反投影函数,3D->2D
 * @param[in]       point   三维图像坐标
 * @param[in]       K       相机内参矩阵
 * @param[in|out]   pixel   二维点坐标
 */
Eigen::Vector3d ReverseProjection(Eigen::Vector3d& point, Eigen::Matrix3d& K);

/**
 * @brief 多点相机反投影函数,3D->2D
 * @param[in]       points   三维点集合
 * @param[in]       K        相机内参矩阵
 * @param[in|out]   point    二维点坐标
 */
vector<Eigen::Vector3d> BuckReverseProjection(vector<Eigen::Vector3d>& points, Eigen::Matrix3d& K);

