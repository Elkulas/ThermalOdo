// 用于整合计算两视图几何
// @input 两张16位图片，相机内参，ORB提取参数
// @output 两张图片无尺度的旋转和平移
// 主要整合了ORBSLAM中的特征点提取工作、特征点匹配工作、根据特征点计算相机位姿的工作

#include "RtGenerator.h"

#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBmatcher.h"
#include"ORBextractor.h"

#include<iostream>

namespace ORB_SLAM2
{

// 构造函数
RtGenerator::RtGenerator(int _nfeatures, float _scaleFactor, int _nlevels, int _iniThFAST, int _minThFAST,
                         double _fx, double _fy, double _cx, double _cy):
                         nfeatures(_nfeatures), scaleFactor(_scaleFactor), nlevels(_nlevels), iniThFAST(_iniThFAST),
                         minThFAST(_minThFAST), fx(_fx), fy(_fy), cx(_cx), cy(_cy)
{
    // 设置内参矩阵
    mK = (cv::Mat_<float>(3,3) << _fx, 0, _cx, 0, _fy, _cy, 0, 0, 1);
}

/**
 * @brief 获得两16位视图之间的Rt关系
 * @param[in] ref       参考图像帧
 * @param[in] curr      当前图像帧
 * @param[in|out] Rcw   相机从参考帧到当前帧的旋转
 * @param[in|out] tcw   相机从参考帧到当前帧的平移
 * 
 */
void RtGenerator::GetRt(cv::Mat ref, cv::Mat curr, cv::Mat Rcw, cv::Mat tcw)
{
    // ORB提取参数
    int nFeatures = nfeatures;
    float fScaleFactor = scaleFactor;
    int nLevels = nlevels;
    int fIniThFAST = iniThFAST;
    int fMinThFAST = minThFAST;

    // = = = = = = = = = = = = = 特征点提取 = = = = = = = = = = = = = 
    // 生成参考帧的实例
    ORBextractor ORBref = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    ORBextractor ORBcurr = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
    
    // 提取特征点
    ORBref.operator()(ref, cv::Mat(), ORBref.mvKeysUn, ORBref.mDescriptors);
    ORBcurr.operator()(curr, cv::Mat(), ORBcurr.mvKeysUn, ORBcurr.mDescriptors);

    // 获取提取的点的数目
    ORBref.N = ORBref.mvKeysUn.size();
    ORBcurr.N = ORBcurr.mvKeysUn.size();

    // 计算图像边界
    ORBref.ComputeImageBounds(ref);
    ORBcurr.ComputeImageBounds(curr);

    // 计算网格参数
    ORBref.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(ORBref.mnMaxX-ORBref.mnMinX);
    ORBref.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(ORBref.mnMaxY-ORBref.mnMinY);
    ORBcurr.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(ORBcurr.mnMaxX-ORBcurr.mnMinX);
    ORBcurr.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(ORBcurr.mnMaxY-ORBcurr.mnMinY);

    // 将特征点对应到网格
    ORBref.AssignFeaturesToGrid();
    ORBcurr.AssignFeaturesToGrid();

    // = = = = = = = = = = = = = 寻找匹配关系 = = = = = = = = = = = = = 
    // 构建匹配的实例
    ORBmatcher matcher(
        0.9,                // 最佳点和次佳特征点评分的比值阈值。此处比较宽松，tracking时一般是0.7
        true                // 检查特征点的方向
        );
    
    // 存储匹配关系的容器
    std::vector<int> Matches12;
    // 特征点类型从Keypoints转换为Points2d，内容为参考帧中的内容
    std::vector<cv::Point2f> PrevMatched;
    PrevMatched.resize(ORBref.mvKeysUn.size());
    for(int i = 0; i<ORBref.mvKeysUn.size(); i++)
        PrevMatched[i] = ORBref.mvKeysUn[i].pt;

    // 进行特征点匹配操作，输出匹配的个数
    int nmatches = matcher.SearchForInitialization(
        ORBref,                      // 初始化时的参考帧
        ORBcurr,                     // 初始化时的当前帧
        PrevMatched,                // 在初始化参考帧中提取得到的特征点
        Matches12,                  // 保存匹配的关系
        200                         // 搜索窗口大小
        );
    //TODO: 将match转变为DMatch，在图像中进行作图

    // = = = = = = = = = = = = = 计算H和F并解算旋转与平移 = = = = = = = = = = = = = 
    vector<bool> vbTriangulated;
    if(matcher.Initialize(ORBref, ORBcurr, Matches12, Rcw, tcw, matcher.mvIniP3D, vbTriangulated, mK, 1.0, 200))
        cout << "Calculating Rt Over!" << endl;
    cout << "R: " << endl << Rcw << endl;
    cout << "t: " << endl << tcw << endl;
    
}

/**
 * @brief 使用OpenCV的方式计算FAST
 * @param[in] image16       输入的16-bit图像
 * @param[in] image8        输入的8-bit图像
 * @param[in] threshold     FAST提取的阈值
 * 
 */
void RtGenerator::ComputeFAST(const cv::Mat &image16, const cv::Mat &image8, int threshold)
{
    vector<cv::KeyPoint> keypoints, keypoints_flip;
    cv::Mat image16_flip;
    cv::flip(image16, image16_flip, 1);

    cv::FAST(image16, keypoints, threshold);
    cv::FAST(image16_flip, keypoints_flip, threshold);

    // 获得左半边的角点
    for (auto &iter:keypoints)
    {
        iter.pt.x = iter.pt.x/2;
    }
    //获得右半边的角点
    for(auto &iter:keypoints_flip)
    {
        iter.pt.x = image16.cols - iter.pt.x/2;
        keypoints.push_back(iter);
    }

    cout << "OpenCV FAST points number is " << keypoints.size() << endl;

    // 显示
    cv::Mat outshow;
    cv::drawKeypoints(image8, keypoints, outshow);
    cv::imshow("OpenCV keypoints", outshow);
    cv::waitKey(0);
    

}


} //namespace ORB_SLAM2