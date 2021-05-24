// rt生成工具

#ifndef RTGENERATOR_H
#define RTGENERATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"ORBextractor.h"

#include<iostream>

namespace ORB_SLAM2
{
class ORBextractor;
class ORBmatcher;

class RtGenerator
{
public:
    /**
     * @brief Constructor
     * ORB提取所需要的参数
     * @param[in] nfeatures         指定要提取出来的特征点数目
     * @param[in] scaleFactor       图像金字塔的缩放系数
     * @param[in] nlevels           指定需要提取特征点的图像金字塔层
     * @param[in] iniThFAST         初始的默认FAST响应值阈值
     * @param[in] minThFAST         较小的FAST响应值阈值 
     * @param[in] fx,fy,cx,cy       相机内参数
     * 
     */
    RtGenerator(int nfeatures=1000, float scaleFactor=1.2, int nlevels=8, int iniThFAST=25, int minThFAST=5,
    double fx=1073.71767, double fy=1086.81080, double cx=319.89664, double cy=249.62074);

    /** @brief destructor */
    ~RtGenerator(){};

    /**
     * @brief 获得两16位视图之间的Rt关系
     * @param[in] ref       参考图像帧
     * @param[in] curr      当前图像帧
     * @param[in|out] Rcw   相机从参考帧到当前帧的旋转
     * @param[in|out] tcw   相机从参考帧到当前帧的平移
     * 
     */
    void GetRt(cv::Mat ref, cv::Mat curr, cv::Mat& Rcw, cv::Mat& tcw);

    /**
     * @brief 使用OpenCV的方式计算FAST
     * @param[in] image16       输入的16-bit图像
     * @param[in] image8        输入的8-bit图像
     * @param[in] threshold     FAST提取的阈值
     * 
     */
    void ComputeFAST(const cv::Mat &image16, const cv::Mat &image8, int threshold);

        
    int nfeatures;			                    ///<整个图像金字塔中，要提取的特征点数目
    double scaleFactor;		                    ///<图像金字塔层与层之间的缩放因子
    int nlevels;			                    ///<图像金字塔的层数
    int iniThFAST;			                    ///<初始的FAST响应值阈值
    int minThFAST;			                    ///<最小的FAST响应值阈值

    // 相机内参数
    double fx;
    double fy;
    double cx;
    double cy;
    cv::Mat mK;

protected:


};//class RtGenerator

} // namespace ORB_SLAM2

#endif