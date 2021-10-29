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

#include "ORBmatcher.h"
#include<iostream>
#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<stdint-gcc.h>

using namespace std;

namespace ORB_SLAM2
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}


/**
 * @brief 单目初始化中用于参考帧和当前帧的特征点匹配
 * 步骤
 * Step 1 构建旋转直方图
 * Step 2 在半径窗口内搜索当前帧F2中所有的候选匹配特征点 
 * Step 3 遍历搜索搜索窗口中的所有潜在的匹配候选点，找到最优的和次优的
 * Step 4 对最优次优结果进行检查，满足阈值、最优/次优比例，删除重复匹配
 * Step 5 计算匹配点旋转角度差所在的直方图
 * Step 6 筛除旋转直方图中“非主流”部分
 * Step 7 将最后通过筛选的匹配好的特征点保存
 * @param[in] F1                        初始化参考帧                  
 * @param[in] F2                        当前帧
 * @param[in & out] vbPrevMatched       本来存储的是参考帧的所有特征点坐标，该函数更新为匹配好的当前帧的特征点坐标
 * @param[in & out] vnMatches12         保存参考帧F1中特征点是否匹配上，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
 * @param[in] windowSize                搜索窗口
 * @return int                          返回成功匹配的特征点数目
 */
int ORBmatcher::SearchForInitialization(ORBextractor &F1, ORBextractor &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize)
{
    int nmatches=0;

    // F1中特征点和F2中的匹配关系，是按照F1中的特征点数目来进行空间分配
    vnMatches12 = vector<int>(F1.mvKeysUn.size(),-1);

    // Step 1: 构建旋转直方图 HISIO_LENGTH = 30
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    
    // TODO: 此处可能有问题
    const float factor = 1.0f/HISTO_LENGTH;

    // 匹配点对的距离，按照F2特征点数目分配空间
    vector<int> vMatchedDistance(F2.mvKeysUn.size(),INT_MAX);
    // 从帧2到帧1的一个反向匹配，按照F2特征点数目进行空间的分配
    vector<int> vnMatches21(F2.mvKeysUn.size(),-1);

    // 遍历帧1中所有的特征点
    for(size_t i1=0, iend1=F1.mvKeysUn.size(); i1<iend1; i1++)
    {
        // 取出一个F1上的特征点
        cv::KeyPoint kp1 = F1.mvKeysUn[i1];
        int level1 = kp1.octave;
        // 只使用原始图像上提取的特征点
        if(level1>0)
            continue;

        // Step 2: 在半径窗口内搜索当前帧F2内所有的候选匹配特征点
        // vbPrevMatched 输入参考帧 F1中的特征点
        // window = 100, 输入最小最大金字塔层级均为0 
        vector<size_t> vIndices2 = F2.GetFeaturesInArea(vbPrevMatched[i1].x,vbPrevMatched[i1].y, windowSize,level1,level1);

        if(vIndices2.empty())
            continue;

        // 取出参考帧F1中当前遍历特征点对应的描述子
        cv::Mat d1 = F1.mDescriptors.row(i1);

        // 最佳的距离
        int bestDist = INT_MAX;
        // 次佳的距离
        int bestDist2 = INT_MAX;
        int bestIdx2 = -1;

        // Step 3: 遍历搜索搜索窗口中的所有潜在的匹配候选点，找到最优的和次优的
        for(vector<size_t>::iterator vit=vIndices2.begin(); vit!=vIndices2.end(); vit++)
        {
            size_t i2 = *vit;

            // 取出候选特征点所对应的描述子
            cv::Mat d2 = F2.mDescriptors.row(i2);

            // 计算两个特征点描述子之间的距离
            int dist = DescriptorDistance(d1,d2);

            if(vMatchedDistance[i2]<=dist)
                continue;
            
            // 如果当前距离小于已有的最小距离，那么更新最佳和次佳的距离
            if(dist<bestDist)
            {
                bestDist2=bestDist;
                bestDist=dist;
                bestIdx2=i2;
            }
            else if(dist<bestDist2)
            {
                bestDist2=dist;
            }
        }

        // Step 4: 对最优次优结果进行检查，满足阈值、最优/次优的比例，删除重复匹配
        // 即使算出了最佳描述子匹配距离，也不一定保证配对成功，需要小于设定的阈值
        if(bestDist<=TH_LOW)
        {
            // 最佳距离比次佳距离要小于设定的比例
            if(bestDist<(float)bestDist2*mfNNratio)
            {
                // 如果找到的候选特征点对应F1中特征点已经匹配过了，说明发生了重复匹配，将原来的匹配也删掉
                if(vnMatches21[bestIdx2]>=0)
                {
                    vnMatches12[vnMatches21[bestIdx2]]=-1;
                    nmatches--;
                }

                // 次优的匹配关系，双向建立
                // vnMatches12保存参考帧F1和F2匹配关系，index保存是F1对应特征点索引，值保存的是匹配好的F2特征点索引
                vnMatches12[i1]=bestIdx2;
                vnMatches21[bestIdx2]=i1;
                vMatchedDistance[bestIdx2]=bestDist;
                nmatches++;
                
                // Step 5: 计算匹配点旋转角度差所在的直方图
                if(mbCheckOrientation)
                {
                    // 计算匹配特征点的角度差，这里单位是角度不是弧度
                    float rot = F1.mvKeysUn[i1].angle-F2.mvKeysUn[bestIdx2].angle;
                    if(rot<0.0)
                        rot+=360.0f;
                    
                    // 将角度差放置到bin中
                    int bin = round(rot*factor);
                    if(bin==HISTO_LENGTH)
                        bin=0;
                    assert(bin>=0 && bin<HISTO_LENGTH);
                    rotHist[bin].push_back(i1);
                }
            }
        }

    }

    // Step 6: 筛除旋转直方图中“非主流”的部分
    // 角度一致性的检测，保证匹配点对方向上的一致，而不是会出现那种角度差别较大的情况
    if(mbCheckOrientation)
    {
        int ind1=-1;
        int ind2=-1;
        int ind3=-1;

        // 筛选出在旋转角度差落在直方图区间内数量最多的前三个bin 的索引
        ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

        for(int i=0; i<HISTO_LENGTH; i++)
        {
            if(i==ind1 || i==ind2 || i==ind3)
                continue;
            
            // 删除掉那些不在前三bin中的匹配对，因为他们不符合主流的方向
            for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
            {
                int idx1 = rotHist[i][j];
                if(vnMatches12[idx1]>=0)
                {
                    vnMatches12[idx1]=-1;
                    nmatches--;
                }
            }
        }

    }

    //Update prev matched
    // Step 7: 将最后通过筛选的匹配好的特征点保存到vbPrevMatched，也就是将保存好的参考帧上的特征点换成当前帧上的特征点
    for(size_t i1=0, iend1=vnMatches12.size(); i1<iend1; i1++)
        if(vnMatches12[i1]>=0)
            vbPrevMatched[i1]=F2.mvKeysUn[vnMatches12[i1]].pt;

    return nmatches;
}

void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

// = = = = = = = = = = = = = = = Initializer = = = = = = = = = = = = = = = 
/**
 * @brief 计算基础矩阵和单应性矩阵，选取最佳的来恢复出最开始两帧之间的相对姿态，并进行三角化得到初始地图点
 * Step 1 重新记录特征点对的匹配关系
 * Step 2 在所有匹配特征点对中随机选择8对匹配特征点为一组，用于估计H矩阵和F矩阵
 * Step 3 计算fundamental 矩阵 和homography 矩阵，为了加速分别开了线程计算 
 * Step 4 计算得分比例来判断选取哪个模型来求位姿R,t
 * 
 * @param[in] CurrentFrame          当前帧，也就是SLAM意义上的第二帧
 * @param[in] vMatches12            当前帧（2）和参考帧（1）图像中特征点的匹配关系
 *                                  vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
 *                                  没有匹配关系的话，vMatches12[i]值为 -1
 * @param[in & out] R21                   相机从参考帧到当前帧的旋转
 * @param[in & out] t21                   相机从参考帧到当前帧的平移
 * @param[in & out] vP3D                  三角化测量之后的三维地图点
 * @param[in & out] vbTriangulated        标记三角化点是否有效，有效为true
 * @return true                     该帧可以成功初始化，返回true
 * @return false                    该帧不满足初始化条件，返回false
 */
bool ORBmatcher::Initialize(ORBextractor &ReferenceFrame, ORBextractor &CurrentFrame, const vector<int> &vMatches12, cv::Mat &R21, cv::Mat &t21,
                             vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, cv::Mat K, float sigma, int iteration)
{
    // Fill structures with current keypoints and matches with reference frame
    // Reference Frame: 1, Current Frame: 2
    // 获取参考帧的特征点(orbex)
    mvKeys1 = ReferenceFrame.mvKeysUn;

    // 获取当前帧的特征点（orbex2）
    mvKeys2 = CurrentFrame.mvKeysUn;

    // 获取内参矩阵
    mK = K;
    // 获取sigma
    mSigma = sigma;
    mSigma2 = sigma *sigma;
    // 获取迭代次数、
    mMaxIterations = iteration;

    // mvMatches12记录匹配上的特征点对，记录的是帧2在帧1的匹配索引
    mvMatches12.clear();
    // 空间分配，大小和关键点数目一致
    mvMatches12.reserve(mvKeys2.size());
    // 记录参考帧1中的每个特征点是否有匹配的特征点
    mvbMatched1.resize(mvKeys1.size());

    // Step 1 重新记录特征点对的匹配关系存储在mvMatches12，是否有匹配关系存储在mvbMatched1
    // 将vMatches12(存在冗余) 转化为mvMatches12（只记录匹配关系）
    // 这里也就只是做一个形式上的转换工作
    // 将已经有的转换关系提取出来
    for(size_t i=0, iend=vMatches12.size();i<iend; i++)
    {
        // vMatches12[i]解释：i表示帧1中关键点的索引值，vMatches12[i]的值为帧2的关键点索引值
        // 没有匹配关系则值为-1
        if(vMatches12[i]>=0)
        {
            mvMatches12.push_back(make_pair(i,vMatches12[i]));
            mvbMatched1[i]=true;
        }
        else
            mvbMatched1[i]=false;
    }

    std::cout << "CheckPoint1" << std::endl;
    // 有匹配的特征点对数目
    // 此时mvMatches12已经保存了两帧之间所有的有效匹配关系
    const int N = mvMatches12.size();

    // Indices for minimum set selection
    // 新建一个容器来存储特征点索引，并预分配空间
    vector<size_t> vAllIndices;
    vAllIndices.reserve(N);

    // 在RANSAC的某次迭代中，还可以被抽取用来作为数据样本的特征点对的索引
    // 所以叫做可用索引
    vector<size_t> vAvailableIndices;

    // 初始化所有点对的索引
    for(int i=0; i<N; i++)
    {
        vAllIndices.push_back(i);
    }

    // Generate sets of 8 points for each RANSAC iteration
    // Step 2: 在所有匹配特征点对中随机选择8对匹配特征点为一组，用来估计初始H矩阵以及F矩阵
    // 一共选择mMaxIterations组，也就是默认200组
    mvSets = vector< vector<size_t> >(mMaxIterations,vector<size_t>(8,0));

    // 随机数seed
    // 原函数使用dbow2，改写为cv::RNG
    cv::RNG rng(0);

    // 开始迭代
    for(int it=0; it<mMaxIterations; it++)
    {
        // 迭代开始时所有点都是可以用的
        vAvailableIndices = vAllIndices;

        // Select a minimum set
        // 从可用集中选取八个点作为最小的一个数据集
        for(size_t j=0; j<8; j++)
        {
            // 随机产生一对点的id，范围从0 - (N-1)
            int randi = rng.uniform(0,vAvailableIndices.size()-1);
            // idx表示哪一个索引对应的特征点对被选中
            int idx = vAvailableIndices[randi];

            // 将本次迭代中这个选中的第j个特征点对的索引添加到mvSets中
            // 注意这里存的是索引
            // mvSets 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点,实际上是八对
            mvSets[it][j] = idx;

            // 由于这对点在本次迭代中已经被使用了,所以我们为了避免再次抽到这个点,就在"点的可选列表"中,
            // 将这个点原来所在的位置用vector最后一个元素的信息覆盖,并且删除尾部的元素
            // 这样就相当于将这个点的信息从"点的可用列表"中直接删除了
            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        } // 依次提取8个特征点对
    } // 迭代mMaxIterations次，获得mMaxIterations个最小的数据集，也就是八个点
    std::cout << "CheckPoint2 " <<mvSets.size()<< std::endl;
    
    // Launch threads to compute in parallel a fundamental matrix and a homography
    // Step 3 计算fundamental 矩阵 和homography 矩阵，为了加速分别开了线程计算 

    // 这两个变量用于标记在H和F的计算中哪些特征点对被认为是Inlier
    vector<bool> vbMatchesInliersH, vbMatchesInliersF;
    // HF的打分
    float SH, SF;
    // 这两个是经过RANSAC算法后计算出来的单应矩阵和基础矩阵
    cv::Mat H, F;

    // 计算H矩阵及其得分
    // 源代码使用多线程
    FindHomography(vbMatchesInliersH, SH, H);
    std::cout << "CheckPoint3 " << SH << endl << H << std::endl;
    FindFundamental(vbMatchesInliersF, SF, F);
    std::cout << "CheckPoint4 " << SF << endl << F << std::endl;
    // Compute ratio of scores
    //通过这个规则来判断谁的评分占比更多一些，注意不是简单的比较绝对评分大小，而是看评分的占比
    float RH = SH/(SH+SF);

    // Try to reconstruct from homography or fundamental depending on the ratio (0.40-0.45)
    // 注意这里更倾向于用H矩阵恢复位姿。如果单应矩阵的评分占比达到了0.4以上,则从单应矩阵恢复运动,否则从基础矩阵恢复运动
    // if(RH>0.40)
    //     //更偏向于平面，此时从单应矩阵恢复，函数ReconstructH返回bool型结果
    //     return ReconstructH(vbMatchesInliersH,H,mK,R21,t21,vP3D,vbTriangulated,1.0,50);
    // else //if(pF_HF>0.6)
    //     // 更偏向于非平面，从基础矩阵恢复
        return ReconstructF(vbMatchesInliersF,F,mK,R21,t21,vP3D,vbTriangulated,1.0,50);

    // 执行到这里说明跑飞了
    return false;
}

/**
 * @brief 计算单应矩阵，假设场景为平面情况下通过前两帧求取Homography矩阵，并得到该模型的评分
 * 原理参考Multiple view geometry in computer vision  P109 算法4.4
 * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
 * Step 2 选择8个归一化之后的点对进行迭代
 * Step 3 八点法计算单应矩阵矩阵
 * Step 4 利用重投影误差为当次RANSAC的结果评分
 * Step 5 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
 * 
 * @param[in & out] vbMatchesInliers          标记是否是外点
 * @param[in & out] score                     计算单应矩阵的得分
 * @param[in & out] H21                       单应矩阵结果
 */
void ORBmatcher::FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21)
{
    // Number of putative matches
	//匹配的特征点对总数
    const int N = mvMatches12.size();

    // Normalize coordinates
    // Step 1 将当前帧和参考帧中的特征点坐标进行归一化，主要是平移和尺度变换
    // 具体来说,就是将mvKeys1和mvKey2归一化到均值为0，一阶绝对矩为1，归一化矩阵分别为T1、T2
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值
    // 归一化矩阵就是把上述归一化的操作用矩阵来表示。这样特征点坐标乘归一化矩阵可以得到归一化后的坐标
   

	//归一化后的参考帧1和当前帧2中的特征点坐标
    vector<cv::Point2f> vPn1, vPn2;
	// 记录各自的归一化矩阵
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);

	//这里求的逆在后面的代码中要用到，辅助进行原始尺度的恢复
    cv::Mat T2inv = T2.inv();

    // Best Results variables
	// 记录最佳评分
    score = 0.0;
	// 取得历史最佳评分时,特征点对的inliers标记
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
	//某次迭代中，参考帧的特征点坐标
    vector<cv::Point2f> vPn1i(8);
	//某次迭代中，当前帧的特征点坐标
    vector<cv::Point2f> vPn2i(8);
	//以及计算出来的单应矩阵、及其逆矩阵
    cv::Mat H21i, H12i;

    // 每次RANSAC记录Inliers与得分
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
	//下面进行每次的RANSAC迭代
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
		// Step 2 选择8个归一化之后的点对进行迭代
        for(size_t j=0; j<8; j++)
        {
			//从mvSets中获取当前次迭代的某个特征点对的索引信息
            int idx = mvSets[it][j];

            // vPn1i和vPn2i为匹配的特征点对的归一化后的坐标
			// 首先根据这个特征点对的索引信息分别找到两个特征点在各自图像特征点向量中的索引，然后读取其归一化之后的特征点坐标
            vPn1i[j] = vPn1[mvMatches12[idx].first];    //first存储在参考帧1中的特征点索引
            vPn2i[j] = vPn2[mvMatches12[idx].second];   //second存储在参考帧1中的特征点索引
        }//读取8对特征点的归一化之后的坐标

		// Step 3 八点法计算单应矩阵
        // 利用生成的8个归一化特征点对, 调用函数 Initializer::ComputeH21() 使用八点法计算单应矩阵  
        // 关于为什么计算之前要对特征点进行归一化，后面又恢复这个矩阵的尺度？
        // 可以在《计算机视觉中的多视图几何》这本书中P193页中找到答案
        // 书中这里说,8点算法成功的关键是在构造解的方称之前应对输入的数据认真进行适当的归一化
   
        cv::Mat Hn = ComputeH21(vPn1i,vPn2i);
        
        // 单应矩阵原理：X2=H21*X1，其中X1,X2 为归一化后的特征点    
        // 特征点归一化：vPn1 = T1 * mvKeys1, vPn2 = T2 * mvKeys2  得到:T2 * mvKeys2 =  Hn * T1 * mvKeys1   
        // 进一步得到:mvKeys2  = T2.inv * Hn * T1 * mvKeys1
        H21i = T2inv*Hn*T1;
		//然后计算逆
        H12i = H21i.inv();

        // Step 4 利用重投影误差为当次RANSAC的结果评分
        currentScore = CheckHomography(H21i, H12i, 			//输入，单应矩阵的计算结果
									   vbCurrentInliers, 	//输出，特征点对的Inliers标记
									   mSigma);				//TODO  测量误差，在Initializer类对象构造的时候，由外部给定的

    
        // Step 5 更新具有最优评分的单应矩阵计算结果,并且保存所对应的特征点对的内点标记
        if(currentScore>score)
        {
			//如果当前的结果得分更高，那么就更新最优计算结果
            H21 = H21i.clone();
			//保存匹配好的特征点对的Inliers标记
            vbMatchesInliers = vbCurrentInliers;
			//更新历史最优评分
            score = currentScore;
        }
    }
}

/**
 * @brief 计算基础矩阵，假设场景为非平面情况下通过前两帧求取Fundamental矩阵，得到该模型的评分
 * Step 1 将当前帧和参考帧中的特征点坐标进行归一化
 * Step 2 选择8个归一化之后的点对进行迭代
 * Step 3 八点法计算基础矩阵矩阵
 * Step 4 利用重投影误差为当次RANSAC的结果评分
 * Step 5 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
 * 
 * @param[in & out] vbMatchesInliers          标记是否是外点
 * @param[in & out] score                     计算基础矩阵得分
 * @param[in & out] F21                       基础矩阵结果
 */
void ORBmatcher::FindFundamental(vector<bool> &vbMatchesInliers, float &score, cv::Mat &F21)
{
    // 计算基础矩阵,其过程和上面的计算单应矩阵的过程十分相似.

    // Number of putative matches
	// 匹配的特征点对总数
    const int N = vbMatchesInliers.size();

    // Normalize coordinates
    // Step 1 将当前帧和参考帧中的特征点坐标进行归一化，主要是平移和尺度变换
    // 具体来说,就是将mvKeys1和mvKey2归一化到均值为0，一阶绝对矩为1，归一化矩阵分别为T1、T2
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值
    // 归一化矩阵就是把上述归一化的操作用矩阵来表示。这样特征点坐标乘归一化矩阵可以得到归一化后的坐标

    vector<cv::Point2f> vPn1, vPn2;
    cv::Mat T1, T2;
    Normalize(mvKeys1,vPn1, T1);
    Normalize(mvKeys2,vPn2, T2);
	// ! 注意这里取的是归一化矩阵T2的转置,因为基础矩阵的定义和单应矩阵不同，两者去归一化的计算也不相同
    cv::Mat T2t = T2.t();

    // Best Results variables
	//最优结果
    score = 0.0;
    vbMatchesInliers = vector<bool>(N,false);

    // Iteration variables
	// 某次迭代中，参考帧的特征点坐标
    vector<cv::Point2f> vPn1i(8);
    // 某次迭代中，当前帧的特征点坐标
    vector<cv::Point2f> vPn2i(8);
    // 某次迭代中，计算的基础矩阵
    cv::Mat F21i;

    // 每次RANSAC记录的Inliers与得分
    vector<bool> vbCurrentInliers(N,false);
    float currentScore;

    // Perform all RANSAC iterations and save the solution with highest score
    // 下面进行每次的RANSAC迭代
    for(int it=0; it<mMaxIterations; it++)
    {
        // Select a minimum set
        // Step 2 选择8个归一化之后的点对进行迭代
        for(int j=0; j<8; j++)
        {
            int idx = mvSets[it][j];

            // vPn1i和vPn2i为匹配的特征点对的归一化后的坐标
			// 首先根据这个特征点对的索引信息分别找到两个特征点在各自图像特征点向量中的索引，然后读取其归一化之后的特征点坐标
            vPn1i[j] = vPn1[mvMatches12[idx].first];        //first存储在参考帧1中的特征点索引
            vPn2i[j] = vPn2[mvMatches12[idx].second];       //second存储在参考帧1中的特征点索引
        }

        // Step 3 八点法计算基础矩阵
        cv::Mat Fn = ComputeF21(vPn1i,vPn2i);

        // 基础矩阵约束：p2^t*F21*p1 = 0，其中p1,p2 为齐次化特征点坐标    
        // 特征点归一化：vPn1 = T1 * mvKeys1, vPn2 = T2 * mvKeys2  
        // 根据基础矩阵约束得到:(T2 * mvKeys2)^t* Hn * T1 * mvKeys1 = 0   
        // 进一步得到:mvKeys2^t * T2^t * Hn * T1 * mvKeys1 = 0
        F21i = T2t*Fn*T1;

        // Step 4 利用重投影误差为当次RANSAC的结果评分
        currentScore = CheckFundamental(F21i, vbCurrentInliers, mSigma);

		// Step 5 更新具有最优评分的基础矩阵计算结果,并且保存所对应的特征点对的内点标记
        if(currentScore>score)
        {
            //如果当前的结果得分更高，那么就更新最优计算结果
            F21 = F21i.clone();
            vbMatchesInliers = vbCurrentInliers;
            score = currentScore;
        }
    }
}

/**
 * @brief 用DLT方法求解单应矩阵H
 * 这里最少用4对点就能够求出来，不过这里为了统一还是使用了8对点求最小二乘解
 * 
 * @param[in] vP1               参考帧中归一化后的特征点
 * @param[in] vP2               当前帧中归一化后的特征点
 * @return cv::Mat              计算的单应矩阵H
 */
cv::Mat ORBmatcher::ComputeH21(
    const vector<cv::Point2f> &vP1, //归一化后的点, in reference frame
    const vector<cv::Point2f> &vP2) //归一化后的点, in current frame
{
    // 基本原理：见附件推导过程：
    // |x'|     | h1 h2 h3 ||x|
    // |y'| = a | h4 h5 h6 ||y|  简写: x' = a H x, a为一个尺度因子
    // |1 |     | h7 h8 h9 ||1|
    // 使用DLT(direct linear tranform)求解该模型
    // x' = a H x 
    // ---> (x') 叉乘 (H x)  = 0  (因为方向相同) (取前两行就可以推导出下面的了)
    // ---> Ah = 0 
    // A = | 0  0  0 -x -y -1 xy' yy' y'|  h = | h1 h2 h3 h4 h5 h6 h7 h8 h9 |
    //     |-x -y -1  0  0  0 xx' yx' x'|
    // 通过SVD求解Ah = 0，A^T*A最小特征值对应的特征向量即为解
    // 其实也就是右奇异值矩阵的最后一列

	//获取参与计算的特征点的数目
    const int N = vP1.size();

    // 构造用于计算的矩阵 A 
    cv::Mat A(2*N,				//行，注意每一个点的数据对应两行
			  9,				//列
			  CV_32F);      	//float数据类型

	// 构造矩阵A，将每个特征点添加到矩阵A中的元素
    for(int i=0; i<N; i++)
    {
		//获取特征点对的像素坐标
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

		//生成这个点的第一行
        A.at<float>(2*i,0) = 0.0;
        A.at<float>(2*i,1) = 0.0;
        A.at<float>(2*i,2) = 0.0;
        A.at<float>(2*i,3) = -u1;
        A.at<float>(2*i,4) = -v1;
        A.at<float>(2*i,5) = -1;
        A.at<float>(2*i,6) = v2*u1;
        A.at<float>(2*i,7) = v2*v1;
        A.at<float>(2*i,8) = v2;

		//生成这个点的第二行
        A.at<float>(2*i+1,0) = u1;
        A.at<float>(2*i+1,1) = v1;
        A.at<float>(2*i+1,2) = 1;
        A.at<float>(2*i+1,3) = 0.0;
        A.at<float>(2*i+1,4) = 0.0;
        A.at<float>(2*i+1,5) = 0.0;
        A.at<float>(2*i+1,6) = -u2*u1;
        A.at<float>(2*i+1,7) = -u2*v1;
        A.at<float>(2*i+1,8) = -u2;

    }

    // 定义输出变量，u是左边的正交矩阵U， w为奇异矩阵，vt中的t表示是右正交矩阵V的转置
    cv::Mat u,w,vt;

	//使用opencv提供的进行奇异值分解的函数
    cv::SVDecomp(A,							//输入，待进行奇异值分解的矩阵
				 w,							//输出，奇异值矩阵
				 u,							//输出，矩阵U
				 vt,						//输出，矩阵V^T
				 cv::SVD::MODIFY_A | 		//输入，MODIFY_A是指允许计算函数可以修改待分解的矩阵，官方文档上说这样可以加快计算速度、节省内存
				     cv::SVD::FULL_UV);		//FULL_UV=把U和VT补充成单位正交方阵

	// 返回最小奇异值所对应的右奇异向量
    // 注意前面说的是右奇异值矩阵的最后一列，但是在这里因为是vt，转置后了，所以是行；由于A有9列数据，故最后一列的下标为8
    return vt.row(8).reshape(0, 			//转换后的通道数，这里设置为0表示是与前面相同
							 3); 			//转换后的行数,对应V的最后一列
}

/**
 * @brief 根据特征点匹配求fundamental matrix（normalized 8点法）
 * 注意F矩阵有秩为2的约束，所以需要两次SVD分解
 * 
 * @param[in] vP1           参考帧中归一化后的特征点
 * @param[in] vP2           当前帧中归一化后的特征点
 * @return cv::Mat          最后计算得到的基础矩阵F
 */
cv::Mat ORBmatcher::ComputeF21(
    const vector<cv::Point2f> &vP1, //归一化后的点, in reference frame
    const vector<cv::Point2f> &vP2) //归一化后的点, in current frame
{
    // 原理详见附件推导
    // x'Fx = 0 整理可得：Af = 0
    // A = | x'x x'y x' y'x y'y y' x y 1 |, f = | f1 f2 f3 f4 f5 f6 f7 f8 f9 |
    // 通过SVD求解Af = 0，A'A最小特征值对应的特征向量即为解

	//获取参与计算的特征点对数
    const int N = vP1.size();

	//初始化A矩阵
    cv::Mat A(N,9,CV_32F); // N*9维

    // 构造矩阵A，将每个特征点添加到矩阵A中的元素
    for(int i=0; i<N; i++)
    {
        const float u1 = vP1[i].x;
        const float v1 = vP1[i].y;
        const float u2 = vP2[i].x;
        const float v2 = vP2[i].y;

        A.at<float>(i,0) = u2*u1;
        A.at<float>(i,1) = u2*v1;
        A.at<float>(i,2) = u2;
        A.at<float>(i,3) = v2*u1;
        A.at<float>(i,4) = v2*v1;
        A.at<float>(i,5) = v2;
        A.at<float>(i,6) = u1;
        A.at<float>(i,7) = v1;
        A.at<float>(i,8) = 1;
    }

    //存储奇异值分解结果的变量
    cv::Mat u,w,vt;

	
    // 定义输出变量，u是左边的正交矩阵U， w为奇异矩阵，vt中的t表示是右正交矩阵V的转置
    cv::SVDecomp(A,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
	// 转换成基础矩阵的形式
    cv::Mat Fpre = vt.row(8).reshape(0, 3); // v的最后一列

    //基础矩阵的秩为2,而我们不敢保证计算得到的这个结果的秩为2,所以需要通过第二次奇异值分解,来强制使其秩为2
    // 对初步得来的基础矩阵进行第2次奇异值分解
    cv::SVDecomp(Fpre,w,u,vt,cv::SVD::MODIFY_A | cv::SVD::FULL_UV);

	// 秩2约束，强制将第3个奇异值设置为0
    w.at<float>(2)=0; 
    
    // 重新组合好满足秩约束的基础矩阵，作为最终计算结果返回 
    return  u*cv::Mat::diag(w)*vt;
}

/**
 * @brief 对给定的homography matrix打分,需要使用到卡方检验的知识
 * 
 * @param[in] H21                       从参考帧到当前帧的单应矩阵
 * @param[in] H12                       从当前帧到参考帧的单应矩阵
 * @param[in] vbMatchesInliers          匹配好的特征点对的Inliers标记
 * @param[in] sigma                     方差，默认为1
 * @return float                        返回得分
 */
float ORBmatcher::CheckHomography(
    const cv::Mat &H21,                 //从参考帧到当前帧的单应矩阵
    const cv::Mat &H12,                 //从当前帧到参考帧的单应矩阵
    vector<bool> &vbMatchesInliers,     //匹配好的特征点对的Inliers标记
    float sigma)                        //估计误差
{

	// 特点匹配个数
    const int N = mvMatches12.size();

	// Step 1 获取从参考帧到当前帧的单应矩阵的各个元素
    const float h11 = H21.at<float>(0,0);
    const float h12 = H21.at<float>(0,1);
    const float h13 = H21.at<float>(0,2);
    const float h21 = H21.at<float>(1,0);
    const float h22 = H21.at<float>(1,1);
    const float h23 = H21.at<float>(1,2);
    const float h31 = H21.at<float>(2,0);
    const float h32 = H21.at<float>(2,1);
    const float h33 = H21.at<float>(2,2);

	// 获取从当前帧到参考帧的单应矩阵的各个元素
    const float h11inv = H12.at<float>(0,0);
    const float h12inv = H12.at<float>(0,1);
    const float h13inv = H12.at<float>(0,2);
    const float h21inv = H12.at<float>(1,0);
    const float h22inv = H12.at<float>(1,1);
    const float h23inv = H12.at<float>(1,2);
    const float h31inv = H12.at<float>(2,0);
    const float h32inv = H12.at<float>(2,1);
    const float h33inv = H12.at<float>(2,2);

	// 给特征点对的Inliers标记预分配空间
    vbMatchesInliers.resize(N);

	// 初始化score值
    float score = 0;

    // 基于卡方检验计算出的阈值（假设测量有一个像素的偏差）
	// 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 5.991;

    //信息矩阵，方差平方的倒数
    const float invSigmaSquare = 1.0/(sigma * sigma);

    // Step 2 通过H矩阵，进行参考帧和当前帧之间的双向投影，并计算起加权最小二乘投影误差
    // H21 表示从img1 到 img2的变换矩阵
    // H12 表示从img2 到 img1的变换矩阵 
    for(int i = 0; i < N; i++)
    {
		// 一开始都默认为Inlier
        bool bIn = true;

		// Step 2.1 提取参考帧和当前帧之间的特征匹配点对
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Step 2.2 计算 img2 到 img1 的重投影误差
        // x2in1 = H12*x2
        // 将图像2中的特征点单应到图像1中
        // |u1|   |h11inv h12inv h13inv||u2|   |u2in1|
        // |v1| = |h21inv h22inv h23inv||v2| = |v2in1| * w2in1inv
        // |1 |   |h31inv h32inv h33inv||1 |   |  1  |
		// 计算投影归一化坐标
        const float w2in1inv = 1.0/(h31inv * u2 + h32inv * v2 + h33inv);
        const float u2in1 = (h11inv * u2 + h12inv * v2 + h13inv) * w2in1inv;
        const float v2in1 = (h21inv * u2 + h22inv * v2 + h23inv) * w2in1inv;
   
        // 计算重投影误差 = ||p2(i) - H21 * p1(i)||2
        const float squareDist1 = (u1 - u2in1) * (u1 - u2in1) + (v1 - v2in1) * (v1 - v2in1);
        const float chiSquare1 = squareDist1 * invSigmaSquare;

        // Step 2.3 用阈值标记离群点，内点的话累加得分
        if(chiSquare1>th)
            bIn = false;    
        else
            // 误差越大，得分越低
            score += th - chiSquare1;

        // 计算从img1 到 img2 的投影变换误差
        // x1in2 = H21*x1
        // 将图像2中的特征点单应到图像1中
        // |u2|   |h11 h12 h13||u1|   |u1in2|
        // |v2| = |h21 h22 h23||v1| = |v1in2| * w1in2inv
        // |1 |   |h31 h32 h33||1 |   |  1  |
		// 计算投影归一化坐标
        const float w1in2inv = 1.0/(h31*u1+h32*v1+h33);
        const float u1in2 = (h11*u1+h12*v1+h13)*w1in2inv;
        const float v1in2 = (h21*u1+h22*v1+h23)*w1in2inv;

        // 计算重投影误差 
        const float squareDist2 = (u2-u1in2)*(u2-u1in2)+(v2-v1in2)*(v2-v1in2);
        const float chiSquare2 = squareDist2*invSigmaSquare;
 
        // 用阈值标记离群点，内点的话累加得分
        if(chiSquare2>th)
            bIn = false;
        else
            score += th - chiSquare2;   

        // Step 2.4 如果从img2 到 img1 和 从img1 到img2的重投影误差均满足要求，则说明是Inlier point
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    return score;
}

/**
 * @brief 对给定的Fundamental matrix打分
 * 
 * @param[in] F21                       当前帧和参考帧之间的基础矩阵
 * @param[in] vbMatchesInliers          匹配的特征点对属于inliers的标记
 * @param[in] sigma                     方差，默认为1
 * @return float                        返回得分
 */
float ORBmatcher::CheckFundamental(
    const cv::Mat &F21,             //当前帧和参考帧之间的基础矩阵
    vector<bool> &vbMatchesInliers, //匹配的特征点对属于inliers的标记
    float sigma)                    //方差
{

    // 说明：在已值n维观测数据误差服从N(0，sigma）的高斯分布时
    // 其误差加权最小二乘结果为  sum_error = SUM(e(i)^T * Q^(-1) * e(i))
    // 其中：e(i) = [e_x,e_y,...]^T, Q维观测数据协方差矩阵，即sigma * sigma组成的协方差矩阵
    // 误差加权最小二次结果越小，说明观测数据精度越高
    // 那么，score = SUM((th - e(i)^T * Q^(-1) * e(i)))的分数就越高
    // 算法目标：检查基础矩阵
    // 检查方式：利用对极几何原理 p2^T * F * p1 = 0
    // 假设：三维空间中的点 P 在 img1 和 img2 两图像上的投影分别为 p1 和 p2（两个为同名点）
    //   则：p2 一定存在于极线 l2 上，即 p2*l2 = 0. 而l2 = F*p1 = (a, b, c)^T
    //      所以，这里的误差项 e 为 p2 到 极线 l2 的距离，如果在直线上，则 e = 0
    //      根据点到直线的距离公式：d = (ax + by + c) / sqrt(a * a + b * b)
    //      所以，e =  (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)

    // 算法流程
    // input: 基础矩阵 F 左右视图匹配点集 mvKeys1
    //    do:
    //        for p1(i), p2(i) in mvKeys:
    //           l2 = F * p1(i)
    //           l1 = p2(i) * F
    //           error_i1 = dist_point_to_line(x2,l2)
    //           error_i2 = dist_point_to_line(x1,l1)
    //           
    //           w1 = 1 / sigma / sigma
    //           w2 = 1 / sigma / sigma
    // 
    //           if error1 < th
    //              score +=   thScore - error_i1 * w1
    //           if error2 < th
    //              score +=   thScore - error_i2 * w2
    // 
    //           if error_1i > th or error_2i > th
    //              p1(i), p2(i) are inner points
    //              vbMatchesInliers(i) = true
    //           else 
    //              p1(i), p2(i) are outliers
    //              vbMatchesInliers(i) = false
    //           end
    //        end
    //   output: score, inliers

	// 获取匹配的特征点对的总对数
    const int N = mvMatches12.size();

	// Step 1 提取基础矩阵中的元素数据
    const float f11 = F21.at<float>(0,0);
    const float f12 = F21.at<float>(0,1);
    const float f13 = F21.at<float>(0,2);
    const float f21 = F21.at<float>(1,0);
    const float f22 = F21.at<float>(1,1);
    const float f23 = F21.at<float>(1,2);
    const float f31 = F21.at<float>(2,0);
    const float f32 = F21.at<float>(2,1);
    const float f33 = F21.at<float>(2,2);

	// 预分配空间
    vbMatchesInliers.resize(N);

	// 设置评分初始值（因为后面需要进行这个数值的累计）
    float score = 0;

    // 基于卡方检验计算出的阈值
	// 自由度为1的卡方分布，显著性水平为0.05，对应的临界阈值
    const float th = 3.841;

    // 自由度为2的卡方分布，显著性水平为0.05，对应的临界阈值
    const float thScore = 5.991;

	// 信息矩阵，或 协方差矩阵的逆矩阵
    const float invSigmaSquare = 1.0/(sigma*sigma);


    // Step 2 计算img1 和 img2 在估计 F 时的score值
    for(int i=0; i<N; i++)
    {
		//默认为这对特征点是Inliers
        bool bIn = true;

	    // Step 2.1 提取参考帧和当前帧之间的特征匹配点对
        const cv::KeyPoint &kp1 = mvKeys1[mvMatches12[i].first];
        const cv::KeyPoint &kp2 = mvKeys2[mvMatches12[i].second];

		// 提取出特征点的坐标
        const float u1 = kp1.pt.x;
        const float v1 = kp1.pt.y;
        const float u2 = kp2.pt.x;
        const float v2 = kp2.pt.y;

        // Reprojection error in second image
        // Step 2.2 计算 img1 上的点在 img2 上投影得到的极线 l2 = F21 * p1 = (a2,b2,c2)
		const float a2 = f11*u1+f12*v1+f13;
        const float b2 = f21*u1+f22*v1+f23;
        const float c2 = f31*u1+f32*v1+f33;
    
        // Step 2.3 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num2 = a2*u2+b2*v2+c2;
        const float squareDist1 = num2*num2/(a2*a2+b2*b2);
        // 带权重误差
        const float chiSquare1 = squareDist1*invSigmaSquare;
		
        // Step 2.4 误差大于阈值就说明这个点是Outlier 
        // ? 为什么判断阈值用的 th（1自由度），计算得分用的thScore（2自由度）
        // ? 可能是为了和CheckHomography 得分统一？
        if(chiSquare1>th)
            bIn = false;
        else
            // 误差越大，得分越低
            score += thScore - chiSquare1;

        // 计算img2上的点在 img1 上投影得到的极线 l1= p2 * F21 = (a1,b1,c1)
        const float a1 = f11*u2+f21*v2+f31;
        const float b1 = f12*u2+f22*v2+f32;
        const float c1 = f13*u2+f23*v2+f33;

        // 计算误差 e = (a * p2.x + b * p2.y + c) /  sqrt(a * a + b * b)
        const float num1 = a1*u1+b1*v1+c1;
        const float squareDist2 = num1*num1/(a1*a1+b1*b1);

        // 带权重误差
        const float chiSquare2 = squareDist2*invSigmaSquare;

        // 误差大于阈值就说明这个点是Outlier 
        if(chiSquare2>th)
            bIn = false;
        else
            score += thScore - chiSquare2;
        
        // Step 2.5 保存结果
        if(bIn)
            vbMatchesInliers[i]=true;
        else
            vbMatchesInliers[i]=false;
    }
    //  返回评分
    return score;
}

/**
 * @brief 归一化特征点到同一尺度，作为后续normalize DLT的输入
 *  [x' y' 1]' = T * [x y 1]' 
 *  归一化后x', y'的均值为0，sum(abs(x_i'-0))=1，sum(abs((y_i'-0))=1
 *
 *  为什么要归一化？
 *  在相似变换之后(点在不同的坐标系下),他们的单应性矩阵是不相同的
 *  如果图像存在噪声,使得点的坐标发生了变化,那么它的单应性矩阵也会发生变化
 *  我们采取的方法是将点的坐标放到同一坐标系下,并将缩放尺度也进行统一 
 *  对同一幅图像的坐标进行相同的变换,不同图像进行不同变换
 *  缩放尺度是为了让噪声对于图像的影响在一个数量级上
 * 
 *  Step 1 计算特征点X,Y坐标的均值 
 *  Step 2 计算特征点X,Y坐标离均值的平均偏离程度
 *  Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1 
 *  Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
 * 
 * @param[in] vKeys                               待归一化的特征点
 * @param[in & out] vNormalizedPoints             特征点归一化后的坐标
 * @param[in & out] T                             归一化特征点的变换矩阵
 */
void ORBmatcher::Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T)                           //将特征点归一化的矩阵
{
    // 归一化的是这些点在x方向和在y方向上的一阶绝对矩（随机变量的期望）。

    // Step 1 计算特征点X,Y坐标的均值 meanX, meanY
    float meanX = 0;
    float meanY = 0;

	//获取特征点的数量
    const int N = vKeys.size();

	//设置用来存储归一后特征点的向量大小，和归一化前保持一致
    vNormalizedPoints.resize(N);

	//开始遍历所有的特征点
    for(int i=0; i<N; i++)
    {
		//分别累加特征点的X、Y坐标
        meanX += vKeys[i].pt.x;
        meanY += vKeys[i].pt.y;
    }

    //计算X、Y坐标的均值
    meanX = meanX/N;
    meanY = meanY/N;

    // Step 2 计算特征点X,Y坐标离均值的平均偏离程度 meanDevX, meanDevY，注意不是标准差
    float meanDevX = 0;
    float meanDevY = 0;

    // 将原始特征点减去均值坐标，使x坐标和y坐标均值分别为0
    for(int i=0; i<N; i++)
    {
        vNormalizedPoints[i].x = vKeys[i].pt.x - meanX;
        vNormalizedPoints[i].y = vKeys[i].pt.y - meanY;

		//累计这些特征点偏离横纵坐标均值的程度
        meanDevX += fabs(vNormalizedPoints[i].x);
        meanDevY += fabs(vNormalizedPoints[i].y);
    }

    // 求出平均到每个点上，其坐标偏离横纵坐标均值的程度；将其倒数作为一个尺度缩放因子
    meanDevX = meanDevX/N;
    meanDevY = meanDevY/N;
    float sX = 1.0/meanDevX;
    float sY = 1.0/meanDevY;

    // Step 3 将x坐标和y坐标分别进行尺度归一化，使得x坐标和y坐标的一阶绝对矩分别为1 
    // 这里所谓的一阶绝对矩其实就是随机变量到取值的中心的绝对值的平均值（期望）
    for(int i=0; i<N; i++)
    {
		//对，就是简单地对特征点的坐标进行进一步的缩放
        vNormalizedPoints[i].x = vNormalizedPoints[i].x * sX;
        vNormalizedPoints[i].y = vNormalizedPoints[i].y * sY;
    }

    // Step 4 计算归一化矩阵：其实就是前面做的操作用矩阵变换来表示而已
    // |sX  0  -meanx*sX|
    // |0   sY -meany*sY|
    // |0   0      1    |
    T = cv::Mat::eye(3,3,CV_32F);
    T.at<float>(0,0) = sX;
    T.at<float>(1,1) = sY;
    T.at<float>(0,2) = -meanX*sX;
    T.at<float>(1,2) = -meanY*sY;
}

//从F恢复R t
bool ORBmatcher::ReconstructF(
    vector<bool> &vbMatchesInliers, //匹配好的特征点对的Inliers标记
    cv::Mat &F21,                   //从参考帧到当前帧的基础矩阵
    cv::Mat &K,                     //相机的内参数矩阵
    cv::Mat &R21,                   //计算好的相机从参考帧到当前帧的旋转
    cv::Mat &t21,                   //计算好的相机从参考帧到当前帧的平移
    vector<cv::Point3f> &vP3D,      //三角化测量之后的特征点的空间坐标
    vector<bool> &vbTriangulated,   //某个特征点是否被三角化了的标记
    float minParallax,              //认为三角化测量有效的最小视差角
    int minTriangulated)            //认为使用三角化测量进行数据判断的最小测量点数量
{
    
    //  统计有效匹配点个数，并用 N 表示
    //  vbMatchesInliers 中存储匹配点对是否是有效
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i]) N++;
    std::cout << "CheckPoint5 " << std::endl;
    // 根据基础矩阵和相机的内参数矩阵计算本质矩阵
    cv::Mat E21 = K.t()*F21*K;
    std::cout << "CheckPoint6 " << std::endl;
    // 定义本质矩阵分解结果，形成四组解,分别是：
    // (R1, t) (R1, -t) (R2, t) (R2, -t)
    cv::Mat R1, R2, t;

    // 调用解析函数 Initializer::DecomposeE()，从本质矩阵求解两个R解和两个t解，
    // 不过由于两个t解互为相反数，因此这里先只获取一个
    // 虽然这个函数对t有归一化，但并没有决定单目整个SLAM过程的尺度. 
    // 因为 CreateInitialMapMonocular 函数对3D点深度会缩放，然后反过来对 t 有改变.
    DecomposeE(E21,R1,R2,t);  
    cv::Mat t1=t;
    cv::Mat t2=-t;
    std::cout << "CheckPoint7 " << std::endl;
    cout << R1 << endl << R2 << endl << t << endl;

    // TODO: 解决相机焦距过大所产生生成三维点困难的问题，以至于四个解无发生判断好坏 20200831
    R1.copyTo(R21);
    t1.copyTo(t21);

    return true;

    
    // 暂时先将下面部分注释掉
    /*
    // Reconstruct with the 4 hyphoteses and check
    // 从上面求解的4种R和T的组合中，选出最佳组合
    // 原理：若某一组合使恢复得到的3D点位于相机正前方的数量最多，那么该组合就是最佳组合
    // 实现：根据计算的解组合成为四种情况,并依次调用 Initializer::CheckRT() 进行检查,得到可以进行三角化测量的点的数目
	// 定义四组解分别在对同一匹配点集进行三角化测量之后的特征点空间坐标
    vector<cv::Point3f> vP3D1, vP3D2, vP3D3, vP3D4;

	// 定义四组解分别对同一匹配点集的有效三角化结果，True or False
    vector<bool> vbTriangulated1,vbTriangulated2,vbTriangulated3, vbTriangulated4;

	// 定义四种解对应的比较大的特征点对视差角
    float parallax1,parallax2, parallax3, parallax4;

	// 使用同一组匹配点检查四组解，并范围当前解重建的3D点在摄像头前方且投影误差小于阈值的个数，记为有效3D点个数
    int nGood1 = CheckRT(R1,t1,							//当前组解
						 mvKeys1,mvKeys2,				//参考帧和当前帧中的特征点
						 mvMatches12, vbMatchesInliers,	//特征点的匹配关系和Inliers标记
						 K, 							//相机的内参数矩阵
						 vP3D1,							//存储三角化以后特征点的空间坐标
						 4.0*mSigma2,					//三角化测量过程中允许的最大重投影误差
						 vbTriangulated1,				//参考帧中被成功进行三角化测量的特征点的标记
						 parallax1);					//认为某对特征点三角化测量有效的最小视差角
    int nGood2 = CheckRT(R2,t1,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D2, 4.0*mSigma2, vbTriangulated2, parallax2);
    int nGood3 = CheckRT(R1,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D3, 4.0*mSigma2, vbTriangulated3, parallax3);
    int nGood4 = CheckRT(R2,t2,mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K, vP3D4, 4.0*mSigma2, vbTriangulated4, parallax4);

    // 选取最大可三角化测量的点的数目maxGood
    int maxGood = max(nGood1,max(nGood2,max(nGood3,nGood4)));

    cout << "maxgood " << maxGood << endl;
	// 释放变量，并在后面赋值为最佳R和T
    R21 = cv::Mat();
    t21 = cv::Mat();

    // 确定最小的可以三角化的点数为 0.9倍的内点数. 
    // 如果给定的数目笔这个还大,就用大的.
    int nMinGood = max(static_cast<int>(0.9*N), minTriangulated);

	// 统计四组解中能重建有效3D坐标的解个数
    // 此处的有效是指：当前解能重建的有效3D点个数 > 0.7 * maxGood
    int nsimilar = 0;
    if(nGood1>0.7*maxGood)
        nsimilar++;
    if(nGood2>0.7*maxGood)
        nsimilar++;
    if(nGood3>0.7*maxGood)
        nsimilar++;
    if(nGood4>0.7*maxGood)
        nsimilar++;

    // 四个结果中如果没有明显的最优结果或者没有足够数量的三角化点，则返回失败
    // 结果筛选
    // 条件1: 如果四组解能够重建的最多3D点个数仍然小于所要求的3D点个数（mMinGood），则Pass
    // 条件2: 如果存在两组或两组以上的解能有效重建>0.7*maxGood的3D，则Pass，因为存在两个解
    // 在做输入的时候可以不用考虑最优结果与否
    
    if(maxGood<nMinGood || nsimilar>1)	
    {
        std::cout << "CheckPoint8 " << std::endl;
        return false;
    }
    


    // 选择最佳解
    // 条件1: 有效重建最多的3D点，即maxGood == nGoodx，也即是位于相机前方的3D点个数最多
    // 条件2: 3D点重建时的视差角 parallax 必须大于最小视差角 minParallax，理由是角度越大3D点精度越高

    //看看最好的good点是在哪种解的条件下发生的
    if(maxGood==nGood1)
    {
		//如果该种解下的parallax大于函数参数中给定的最小值
        if(parallax1>minParallax)
        {
            // 存储3D坐标
            vP3D = vP3D1;

			// 获取特征点向量的三角化测量标记
            vbTriangulated = vbTriangulated1;

			// 存储相机姿态
            R1.copyTo(R21);
            t1.copyTo(t21);
			
            // 结束
            std::cout << "CheckPoint9 " << std::endl;
            return true;
        }
    }else if(maxGood==nGood2)
    {
        if(parallax2>minParallax)
        {
            vP3D = vP3D2;
            vbTriangulated = vbTriangulated2;

            R2.copyTo(R21);
            t1.copyTo(t21);
            std::cout << "CheckPoint10 " << std::endl;
            return true;
        }
    }else if(maxGood==nGood3)
    {
        if(parallax3>minParallax)
        {
            vP3D = vP3D3;
            vbTriangulated = vbTriangulated3;

            R1.copyTo(R21);
            t2.copyTo(t21);
            std::cout << "CheckPoint11 " << std::endl;
            return true;
        }
    }else if(maxGood==nGood4)
    {
        if(parallax4>minParallax)
        {
            vP3D = vP3D4;
            vbTriangulated = vbTriangulated4;

            R2.copyTo(R21);
            t2.copyTo(t21);
            std::cout << "CheckPoint12 " << std::endl;
            return true;
        }
    }
    */
    // 如果有最优解但是不满足对应的parallax>minParallax，或者是其他的原因导致的无法求出相机R，t，那么返回false表示求解失败
    return false;
}

bool ORBmatcher::ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated)
{
    int N=0;
    for(size_t i=0, iend = vbMatchesInliers.size() ; i<iend; i++)
        if(vbMatchesInliers[i])
            N++;

    // We recover 8 motion hypotheses using the method of Faugeras et al.
    // Motion and structure from motion in a piecewise planar environment.
    // International Journal of Pattern Recognition and Artificial Intelligence, 1988

    cv::Mat invK = K.inv();
    cv::Mat A = invK*H21*K;

    cv::Mat U,w,Vt,V;
    cv::SVD::compute(A,w,U,Vt,cv::SVD::FULL_UV);
    V=Vt.t();

    float s = cv::determinant(U)*cv::determinant(Vt);

    float d1 = w.at<float>(0);
    float d2 = w.at<float>(1);
    float d3 = w.at<float>(2);

    if(d1/d2<1.00001 || d2/d3<1.00001)
    {
        return false;
    }

    vector<cv::Mat> vR, vt, vn;
    vR.reserve(8);
    vt.reserve(8);
    vn.reserve(8);

    //n'=[x1 0 x3] 4 posibilities e1=e3=1, e1=1 e3=-1, e1=-1 e3=1, e1=e3=-1
    float aux1 = sqrt((d1*d1-d2*d2)/(d1*d1-d3*d3));
    float aux3 = sqrt((d2*d2-d3*d3)/(d1*d1-d3*d3));
    float x1[] = {aux1,aux1,-aux1,-aux1};
    float x3[] = {aux3,-aux3,aux3,-aux3};

    //case d'=d2
    float aux_stheta = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1+d3)*d2);

    float ctheta = (d2*d2+d1*d3)/((d1+d3)*d2);
    float stheta[] = {aux_stheta, -aux_stheta, -aux_stheta, aux_stheta};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=ctheta;
        Rp.at<float>(0,2)=-stheta[i];
        Rp.at<float>(2,0)=stheta[i];
        Rp.at<float>(2,2)=ctheta;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=-x3[i];
        tp*=d1-d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }

    //case d'=-d2
    float aux_sphi = sqrt((d1*d1-d2*d2)*(d2*d2-d3*d3))/((d1-d3)*d2);

    float cphi = (d1*d3-d2*d2)/((d1-d3)*d2);
    float sphi[] = {aux_sphi, -aux_sphi, -aux_sphi, aux_sphi};

    for(int i=0; i<4; i++)
    {
        cv::Mat Rp=cv::Mat::eye(3,3,CV_32F);
        Rp.at<float>(0,0)=cphi;
        Rp.at<float>(0,2)=sphi[i];
        Rp.at<float>(1,1)=-1;
        Rp.at<float>(2,0)=sphi[i];
        Rp.at<float>(2,2)=-cphi;

        cv::Mat R = s*U*Rp*Vt;
        vR.push_back(R);

        cv::Mat tp(3,1,CV_32F);
        tp.at<float>(0)=x1[i];
        tp.at<float>(1)=0;
        tp.at<float>(2)=x3[i];
        tp*=d1+d3;

        cv::Mat t = U*tp;
        vt.push_back(t/cv::norm(t));

        cv::Mat np(3,1,CV_32F);
        np.at<float>(0)=x1[i];
        np.at<float>(1)=0;
        np.at<float>(2)=x3[i];

        cv::Mat n = V*np;
        if(n.at<float>(2)<0)
            n=-n;
        vn.push_back(n);
    }


    int bestGood = 0;
    int secondBestGood = 0;    
    int bestSolutionIdx = -1;
    float bestParallax = -1;
    vector<cv::Point3f> bestP3D;
    vector<bool> bestTriangulated;

    // Instead of applying the visibility constraints proposed in the Faugeras' paper (which could fail for points seen with low parallax)
    // We reconstruct all hypotheses and check in terms of triangulated points and parallax
    for(size_t i=0; i<8; i++)
    {
        float parallaxi;
        vector<cv::Point3f> vP3Di;
        vector<bool> vbTriangulatedi;
        int nGood = CheckRT(vR[i],vt[i],mvKeys1,mvKeys2,mvMatches12,vbMatchesInliers,K,vP3Di, 4.0*mSigma2, vbTriangulatedi, parallaxi);

        if(nGood>bestGood)
        {
            secondBestGood = bestGood;
            bestGood = nGood;
            bestSolutionIdx = i;
            bestParallax = parallaxi;
            bestP3D = vP3Di;
            bestTriangulated = vbTriangulatedi;
        }
        else if(nGood>secondBestGood)
        {
            secondBestGood = nGood;
        }
    }


    if(secondBestGood<0.75*bestGood && bestParallax>=minParallax && bestGood>minTriangulated && bestGood>0.9*N)
    {
        vR[bestSolutionIdx].copyTo(R21);
        vt[bestSolutionIdx].copyTo(t21);
        vP3D = bestP3D;
        vbTriangulated = bestTriangulated;

        return true;
    }

    return false;
}


int ORBmatcher::CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<std::pair<int,int>> &vMatches12, vector<bool> &vbMatchesInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax)
{
    // Calibration parameters
    const float fx = K.at<float>(0,0);
    const float fy = K.at<float>(1,1);
    const float cx = K.at<float>(0,2);
    const float cy = K.at<float>(1,2);

    vbGood = vector<bool>(vKeys1.size(),false);
    vP3D.resize(vKeys1.size());

    vector<float> vCosParallax;
    vCosParallax.reserve(vKeys1.size());

    // Camera 1 Projection Matrix K[I|0]
    cv::Mat P1(3,4,CV_32F,cv::Scalar(0));
    K.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat O1 = cv::Mat::zeros(3,1,CV_32F);

    // Camera 2 Projection Matrix K[R|t]
    cv::Mat P2(3,4,CV_32F);
    R.copyTo(P2.rowRange(0,3).colRange(0,3));
    t.copyTo(P2.rowRange(0,3).col(3));
    P2 = K*P2;

    cv::Mat O2 = -R.t()*t;

    int nGood=0;

    for(size_t i=0, iend=vMatches12.size();i<iend;i++)
    {
        if(!vbMatchesInliers[i])
            continue;

        const cv::KeyPoint &kp1 = vKeys1[vMatches12[i].first];
        const cv::KeyPoint &kp2 = vKeys2[vMatches12[i].second];
        cv::Mat p3dC1;

        Triangulate(kp1,kp2,P1,P2,p3dC1);

        if(!isfinite(p3dC1.at<float>(0)) || !isfinite(p3dC1.at<float>(1)) || !isfinite(p3dC1.at<float>(2)))
        {
            vbGood[vMatches12[i].first]=false;
            continue;
        }

        // Check parallax
        cv::Mat normal1 = p3dC1 - O1;
        float dist1 = cv::norm(normal1);

        cv::Mat normal2 = p3dC1 - O2;
        float dist2 = cv::norm(normal2);

        float cosParallax = normal1.dot(normal2)/(dist1*dist2);

        // Check depth in front of first camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        if(p3dC1.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check depth in front of second camera (only if enough parallax, as "infinite" points can easily go to negative depth)
        cv::Mat p3dC2 = R*p3dC1+t;

        if(p3dC2.at<float>(2)<=0 && cosParallax<0.99998)
            continue;

        // Check reprojection error in first image
        float im1x, im1y;
        float invZ1 = 1.0/p3dC1.at<float>(2);
        im1x = fx*p3dC1.at<float>(0)*invZ1+cx;
        im1y = fy*p3dC1.at<float>(1)*invZ1+cy;

        float squareError1 = (im1x-kp1.pt.x)*(im1x-kp1.pt.x)+(im1y-kp1.pt.y)*(im1y-kp1.pt.y);

        if(squareError1>th2)
            continue;

        // Check reprojection error in second image
        float im2x, im2y;
        float invZ2 = 1.0/p3dC2.at<float>(2);
        im2x = fx*p3dC2.at<float>(0)*invZ2+cx;
        im2y = fy*p3dC2.at<float>(1)*invZ2+cy;

        float squareError2 = (im2x-kp2.pt.x)*(im2x-kp2.pt.x)+(im2y-kp2.pt.y)*(im2y-kp2.pt.y);

        if(squareError2>th2)
            continue;

        vCosParallax.push_back(cosParallax);
        vP3D[vMatches12[i].first] = cv::Point3f(p3dC1.at<float>(0),p3dC1.at<float>(1),p3dC1.at<float>(2));
        nGood++;

        if(cosParallax<0.99998)
            vbGood[vMatches12[i].first]=true;
    }

    if(nGood>0)
    {
        sort(vCosParallax.begin(),vCosParallax.end());

        size_t idx = min(50,int(vCosParallax.size()-1));
        parallax = acos(vCosParallax[idx])*180/CV_PI;
    }
    else
        parallax=0;

    return nGood;
}

void ORBmatcher::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x*P1.row(2)-P1.row(0);
    A.row(1) = kp1.pt.y*P1.row(2)-P1.row(1);
    A.row(2) = kp2.pt.x*P2.row(2)-P2.row(0);
    A.row(3) = kp2.pt.y*P2.row(2)-P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    x3D = vt.row(3).t();
    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);
}

void ORBmatcher::DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t)
{
    cv::Mat u,w,vt;
    cv::SVD::compute(E,w,u,vt);

    u.col(2).copyTo(t);
    t=t/cv::norm(t);

    cv::Mat W(3,3,CV_32F,cv::Scalar(0));
    W.at<float>(0,1)=-1;
    W.at<float>(1,0)=1;
    W.at<float>(2,2)=1;

    R1 = u*W*vt;
    if(cv::determinant(R1)<0)
        R1=-R1;

    R2 = u*W.t()*vt;
    if(cv::determinant(R2)<0)
        R2=-R2;
}



} //namespace ORB_SLAM
