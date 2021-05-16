/**
 * @brief 
 * 该文件中的函数主要是关于相机的一些操作
 * 其中包括投影与反投影等
 */

#include "projection.h"

using namespace std;

/**
 * @brief 将内参转换为内参矩阵Ｋ
 * @param[in]       fx      
 * @param[in]       fy      
 * @param[in]       cx      
 * @param[in]       cy
 * @param[out]      K
 */
Eigen::Matrix3d GenerateK(const double& fx, const double& fy, const double& cx, const double& cy)
{
    Eigen::Matrix3d K;
    K << fx, 0, cx, 0, fy, cy, 0, 0, 1;
    return K;   
}

/**
 * @brief 转换三维点坐标
 * @param[in]       points1          原始三维点集
 * @param[in|out]   points2          输出三维点集
 * @param[in]       T21              转换关系 1->2 
 */
void PointsTransfer(vector<Eigen::Vector3d>& points1, vector<Eigen::Vector3d>& points2, Eigen::Isometry3d& T21)
{
    for( auto point:points1)
    {
        Eigen::Vector3d temp;
        temp = T21*point;
        points2.emplace_back(temp);
    }
}

/**
 * @brief 单点相机投影函数,2D->3D
 * @param[in]       pixel   二维图像坐标
 * @param[in]       K       相机内参矩阵
 * @param[in]       depth   该点的深度
 * @param[in|out]   point   三维点坐标
 */
Eigen::Vector3d Projection(Eigen::Vector3d& pixel, const double& depth, Eigen::Matrix3d& K)
{
    // 检查pixel格式
    if(pixel(2)!=1)
    cerr<<"Check pixel format" <<endl;

    Eigen::Vector3d point;
    point = K.inverse() * (pixel * depth);

    return point;
}

/**
 * @brief 多点相机投影函数,2D->3D, 使用深度图
 * @param[in]       pixels   二维图像坐标集合
 * @param[in]       K        相机内参矩阵
 * @param[in]       depth    对应的深度图
 * @param[in|out]   point    三维点坐标
 */
vector<Eigen::Vector3d> BuckProjection(vector<Eigen::Vector3d>& pixels, cv::Mat& depth, Eigen::Matrix3d& K)
{
    vector<Eigen::Vector3d> points;

    for(auto pixel:pixels)
    {
        // 这里默认使用深度的factor为1000
        double d = static_cast<double>(depth.ptr<ushort>((int)(pixel(1)))[(int)pixel(0)])/ 1000;
        Eigen::Vector3d point = Projection(pixel, d, K);
        points.emplace_back(point);
    }

    return points;
}


/**
 * @brief 单点相机反投影函数,3D->2D
 * @param[in]       point   三维图像坐标
 * @param[in]       K       相机内参矩阵
 * @param[in|out]   pixel   二维点坐标
 */
Eigen::Vector3d ReverseProjection(Eigen::Vector3d& point, Eigen::Matrix3d& K)
{
    // 获取depth倒数
    double inver_depth = double(1.0) / point(2);

    Eigen::Vector3d pixel;
    pixel = K * (point * inver_depth);

    return pixel;
}

/**
 * @brief 多点相机反投影函数,3D->2D
 * @param[in]       points   三维点集合
 * @param[in]       K        相机内参矩阵
 * @param[in|out]   point    二维点坐标
 */
vector<Eigen::Vector3d> BuckReverseProjection(vector<Eigen::Vector3d>& points, Eigen::Matrix3d& K)
{
    vector<Eigen::Vector3d> pixels;

    for(auto point:points)
    {
        double d = point(2);
        Eigen::Vector3d pixel = ReverseProjection(point, K);
        pixels.emplace_back(pixel);
    }

    return pixels;
}