
// C++ 标准库
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>

// Eigen库
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

// OpenCV 库
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

// OpenCV 4.0 库
// contrib
#include <opencv2/ximgproc.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace std;

void myHarries( const cv::Mat& src, cv::Mat& eigenv, int block_size, int aperture_size, double k);
/**
 * @brief 计算最大最小值
 * @param[in]   image   输入图像
 * @param[out]  maxmin  最大最小值
 */
Eigen::Vector2d GetMaxAndMin(cv::Mat& image);

/**
 * @brief 将16位图片压缩为8位图片
 * @param[in]   image_16    16位图像
 * @param[out]  image_8     8位图像
 */
cv::Mat Swap16To8(cv::Mat& image_16);

/**
 * @brief 使用Sobel算子计算的方式计算梯度X,8位or16位
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  Gx      x方向梯度   
 */
double PointGradXSobel(cv::Mat& image, int x, int y);

/**
 * @brief 使用Sobel算子计算的方式计算梯度Y,8位or16位
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  Gy      y方向梯度   
 */
double PointGradYSobel(cv::Mat& image, int x, int y);

/**
 * @brief 对整个图进行sobel处理,输出GradX图
 * @param[in]   image   输入图像
 * @param[out]  GradX   输出x方向梯度图像
 */
cv::Mat GetGradXImage(cv::Mat& image);

/**
 * @brief 对整个图进行sobel处理,输出GradX图
 * @param[in]   image   输入图像
 * @param[out]  GradY   输出Y方向梯度图像
 */
cv::Mat GetGradYImage(cv::Mat& image);

/**
 * @brief 对整个图进行sobel处理,输出GradX * GradX图,cv16uc1
 * @param[in]   image    输入图像
 * @param[out]  GradXX   输出X方向梯度平方图像
 */
cv::Mat GetGradXXImage(cv::Mat& image);

/**
 * @brief 对整个图进行sobel处理,输出GradY * GradY图
 * @param[in]   image     输入图像
 * @param[out]  GradYY    输出Y方向梯度平方图像
 */
cv::Mat GetGradYYImage(cv::Mat& image);

/**
 * @brief 对整个图进行sobel处理,输出GradX * GradY图
 * @param[in]   image       输入图像
 * @param[out]  GradXY      输出X方向梯度 * Y方向梯度图像
 */
cv::Mat GetGradXYImage(cv::Mat& image);

/**
 * @brief 对窗口内的XY,XX,YY图像做boxfilter
 * @param[in]   GradXX      Gx*Gx图
 * @param[in]   GradXY      Gx*Gy图
 * @param[in]   GradYY      Gy*Gy图
 * @param[in]   blocksize   滤波的窗口大小
 * @param[in]   normal      是否进行均一化
 */
void BoxFilter3(cv::Mat& GradXX, cv::Mat& GradXY, cv::Mat& GradYY, int blocksize, bool normal);

/**
 * @brief 对整个图进行sobel处理,输出Grad图
 * @param[in]   image   输入图像
 * @param[out]  Grad    输出梯度图像
 */
cv::Mat GetGradImage(cv::Mat& image);

/**
 * @brief 对整个图进行sobel处理,输出Grad图
 * @param[in]   image   输入图像
 * @param[out]  Grad    输出梯度图像
 */
cv::Mat GetGradImageByAdd(cv::Mat& image);

/**
 * @brief 使用Sobel算子计算的方式计算梯度
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  grad    x,y方向   
 */
Eigen::Vector2d PointGradSobel(cv::Mat& image, int x, int y);

/**
 * @brief 计算该点梯度特征值之比
 *        ratio = a1/a2 < 1
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标
 * @param[in]   y       点的纵坐标
 * @param[out]  ratio   点梯度特征值比值
 */
double GetEigenRatio(cv::Mat& image, int x, int y);

/**
 * @brief 计算该点的响应值
 *        R = det(M) - a*trace(M)^2
 * @param[in]   grad    该点的x,y梯度
 * @param[in]   alpha   权重值
 * @param[out]  R       角点响应值
 */
double CornerScore(Eigen::Vector2d& grad, double alpha);

/**
 * @brief 判断图像类型是否为8位
 * @param[in]   image   16位or8位图像
 * @param[out]  res     输出的布尔变量    
 */
bool Check8BitImage(cv::Mat& image);

/**
 * @brief 检查并输出3x3像素格中九个像素
 * @param[in]   image   图像
 * @param[in]   x       点的横坐标 cols
 * @param[in]   y       点的纵坐标 rows
 */
void CheckBlock3x3(cv::Mat& image, int x, int y);

/**
 * @brief 判断图像类型是否为16位
 * @param[in]   image   16位or8位图像
 * @param[out]  res     输出的布尔变量    
 */
bool Check16BitImage(cv::Mat& image);

/**
 * @brief 图像提取FAST
 * @param[in]   image       16位or8位图像
 * @param[in]   threshold   阈值
 * @param[out]  keypoints   特征点输出
 */
vector<cv::KeyPoint> ComputeFAST(cv::Mat& image, const int threshold);

/**
 * @brief 在图像上绘制角点并显示
 * @param[in]   image       图像
 * @param[in]   keypoints   角点
 */
void DrawKeypoints(cv::Mat& image, vector<cv::KeyPoint>& keypoints);

/**
 * @brief 8位灰度图像提取直线特征,使用FastLineDetector
 * @param[in]   image                   原图像
 * @param[in]   length_threshold        直线线段长度阈值
 * @param[in]   distance_threshold      超出假定线的点的阈值
 * @param[in]   canny_th1               First threshold for hysteresis procedure in canny
 * @param[in]   canny_th2               Second threshold for hysteresis procedure in canny
 * @param[in]   canny_aperture_size     Aperturesize for the sobel operator in Canny
 * @param[in]   do_merge                是否对线段进行merge
 * @param[in]   lines                   输出的结果线段
 */
vector<cv::Vec4f> ComputeLine(cv::Mat& image,
                              int length_threshold,
                              float distance_threshold,
                              int canny_th1,
                              int canny_th2,
                              int canny_aperture_size,
                              bool do_merge);


/**
 * @brief 8位灰度图像提取直线特征,使用FastLineDetector
 * @param[in]   image                   原图像
 * @param[in]   length_threshold        直线线段长度阈值
 * @param[in]   distance_threshold      超出假定线的点的阈值
 * @param[in]   canny_th1               First threshold for hysteresis procedure in canny
 * @param[in]   canny_th2               Second threshold for hysteresis procedure in canny
 * @param[in]   canny_aperture_size     Aperturesize for the sobel operator in Canny
 * @param[in]   do_merge                是否对线段进行merge
 * @param[in]   lines                   输出的结果线段
 */
void ComputeAndDrawLine(cv::Mat& image,
                        int length_threshold,
                        float distance_threshold,
                        int canny_th1,
                        int canny_th2,
                        int canny_aperture_size,
                        bool do_merge);


// 
