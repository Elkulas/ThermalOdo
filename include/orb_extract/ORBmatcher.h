/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"ORBextractor.h"

using namespace std;

namespace ORB_SLAM2
{

class ORBmatcher
{    
public:

    ORBmatcher(float nnratio=0.6, bool checkOri=true);

    // Computes the Hamming distance between two ORB descriptors
    static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

    // Matching for the Map Initialization (only used in the monocular case)
    int SearchForInitialization(ORBextractor &F1, ORBextractor &F2, std::vector<cv::Point2f> &vbPrevMatched, std::vector<int> &vnMatches12, int windowSize=10);

    // Initializer.cc中的内容
    // 计算基础矩阵和单应性矩阵，选取最佳的来恢复出最开始两帧之间的相对姿态，并进行三角化得到初始地图点
    bool Initialize(ORBextractor &ReferenceFrame, ORBextractor &CurrentFrame, const vector<int> &vMatches12,
    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, cv::Mat K, float sigma, int iteration);
    
    void FindHomography(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    void FindFundamental(std::vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21);

    // 用DLT方法求解单应矩阵H
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    // 根据特征点匹配求fundamental matrix（normalized 8点法）
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    float CheckHomography(const cv::Mat &H21,const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);
    
    // 归一化
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    // 分解H获得Rt以及初始化的三角点
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21, 
    vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K, cv::Mat &R21, cv::Mat &t21, 
    vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);


    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                const vector<std::pair<int,int>> &vMatches12, vector<bool> &vbMatchesInliers,
                const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);



    /** 存储Reference Frame中的特征点 */
    vector<cv::KeyPoint> mvKeys1; 
    /** 存储Current Frame中的特征点 */
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    /** Match的数据结构是pair,mvMatches12只记录Reference到Current匹配上的特征点对  */
    vector<std::pair<int, int>> mvMatches12;
    /** 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点 */ 
    vector<bool> mvbMatched1;
    /** 相机内参 */
    cv::Mat mK;
    /** 测量误差 */
    float mSigma, mSigma2;
    /** 算Fundamental和Homography矩阵时RANSAC迭代次数  */
    int mMaxIterations; 
    // Ransac sets
    /** 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点,实际上是八对 */
    vector<vector<size_t> > mvSets; 
    // 初始化生成的三维点
    std::vector<cv::Point3f> mvIniP3D;


public:

    static const int TH_LOW;
    static const int TH_HIGH;
    static const int HISTO_LENGTH;


protected:


    void ComputeThreeMaxima(std::vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

    float mfNNratio;            ///< 最优评分和次优评分的比例
    bool mbCheckOrientation;    ///< 是否检查特征点的方向
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
