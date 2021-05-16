#include "localUtils.h"

using namespace std;

/**
 * @brief 生成等距矩阵T,输入为Mat
 * @param[in]       R       rotation
 * @param[in]       t       translation
 * @param[out]      T       等距变换矩阵
 */
Eigen::Isometry3d GenerateT(cv::Mat& R, cv::Mat& t)
{
    Eigen::Matrix3d rotation;
    Eigen::Vector3d trans;
    cv::cv2eigen(t, trans);
    cv::cv2eigen(R, rotation);
    Eigen::Isometry3d T(rotation);
    T.pretranslate(trans);
    return T;
}

/**
 * @brief 将三维关键点根据T来投影到另一个图像平面上,并绘制
 * @param[in]       keypoints       三维的关键点
 * @param[in]       rgb             投影图像
 * @param[in]       T21             三维关键点到图像的变换关系
 * @param[in]       K               图像内参
 */
void MapFrameToFrame(vector<Eigen::Vector3d>& keypoints, cv::Mat& rgb, Eigen::Isometry3d& T21, Eigen::Matrix3d& K)
{
    // 通过外参转移
    Eigen::Matrix4d T21m4d = T21.matrix();

    vector<Eigen::Vector3d> vRGB; 

    for(auto point:keypoints)
    {
        Eigen::Vector3d pRGB = T21m4d.block<3,3>(0,0) * point + T21m4d.block<3,1>(0,3);
        // cout << point.matrix() <<endl <<endl;
        vRGB.push_back(pRGB);
    }
    // Mapping
    vector<cv::KeyPoint> vKeyRGB;
    
    for(auto point:vRGB)
    {
        // 投影
        //cout << point.matrix() << endl;
        double depthRGB = point(2);
        if(depthRGB <= 0.001) continue;

        Eigen::Vector3d pixelRGB = (1 / depthRGB) * K * point;
        // cout << pixelRGB.matrix() <<endl << endl;
        float u = static_cast<float>(pixelRGB(0));
        float v = static_cast<float>(pixelRGB(1));

        //cout << u << " " << v <<endl;
        // 超出图片尺寸范围的去除
        if(u < 0 || u > rgb.cols || v < 0 || v > rgb.rows) continue;

        // 构建cv::Keypoint的temp实例,将满足要求的点存入
        cv::KeyPoint tempkp;
        tempkp.pt.x = float(u);
        tempkp.pt.y = float(v);

        // 存入存放关键点的容器
        vKeyRGB.push_back(tempkp);
    }
    cv::Mat out12;
    cv::drawKeypoints(rgb, vKeyRGB, out12);
    cv::imshow("keypoints on RGB", out12);
}