#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <sstream>
#include <string>

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

using namespace ORB_SLAM2;
using namespace std;

void ComputeFAST(const cv::Mat &image, int &threshold);

int main(int, char**) {

    // = = = = = = = = = 输入路径 = = = = = = = = = 

    string dataset_dir = "/home/jjj/NGCLAB/dataset/GaoDeThermalData/img16mobile/";
    string dir16 = "img_16_cut/"; 
    string dir8 = "img_8_cut/";
    string type = ".png";
    stringstream ss;
    int id_1 = 87;
    int id_2 = 88;

    // 16 directory
    ss << dataset_dir << dir16 << id_1 << type;
    string img_1_16 = ss.str();
    ss.str("");

    ss << dataset_dir << dir8 << id_1 << type;
    string img_1_8 = ss.str();
    ss.str("");

    ss << dataset_dir << dir16 << id_2 << type;
    string img_2_16 = ss.str();
    ss.str("");

    ss << dataset_dir << dir8 << id_2 << type;
    string img_2_8 = ss.str();
    ss.str("");


    cv::Mat src16 = cv::imread(img_1_16,-1);
    cv::Mat src8 = cv::imread(img_1_8,0);

    cv::Mat src162 = cv::imread(img_2_16,-1);
    cv::Mat src82 = cv::imread(img_2_8,0);

    // = = = = = = = = = Load Config = = = = = = = = = 
    // fast提取参数 
    int nFeatures = 1000;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 200;
    int fMinThFAST = 100;
    // 内参矩阵
    // Gaode
    double fx = 4.401844147106322e+02;
    double fy = 4.398887665018974e+02;
    double cx = 3.246975373514975e+02;
    double cy = 2.543902325439119e+02;

    cv::Mat K = (cv::Mat_<float>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
    
    std::cout << K << std::endl;

    // = = = = = = = = = 普通提取FAST = = = = = = = = = 

    // 构造提取器实例
    RtGenerator RtGen = RtGenerator();
    
    RtGen.ComputeFAST(src16, src8, fIniThFAST);


    // = = = = = = = = = ORB计算match = = = = = = = = = 
    cv::Mat Rcw;
    cv::Mat tcw;

    // 主函数，提取两张图片中的Rt
    // RtGen.GetRt(src16, src162, Rcw, tcw);
    
    // = = = = = = = = = = = = = = = = = = = = = = = = 
    
    // 创建提取器
    // 直接创建实例
    // ORBSLAM中的Frame本质上其实就是改编部分的的ORBextractor
    ORBextractor orbex = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    orbex.operator()(src16, cv::Mat(), orbex.mvKeysUn, orbex.mDescriptors);
    // = = = = = = = = = 将特征点分配区域网格 = = = = = = = = =
    // 存放特征点数目
    orbex.N = orbex.mvKeysUn.size();

    // 计算边界
    orbex.ComputeImageBounds(src16);
    // 计算网格参数
    orbex.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(orbex.mnMaxX-orbex.mnMinX);
    orbex.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(orbex.mnMaxY-orbex.mnMinY);
    
    // 将特征点分配到图像网格中
    orbex.AssignFeaturesToGrid();

    // 第二个进行比对的图像
    // 创建第二个实例
    ORBextractor orbex2 = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    orbex2.operator()(src162, cv::Mat(), orbex2.mvKeysUn, orbex2.mDescriptors);
    // = = = = = = = = = 将特征点分配区域网格 = = = = = = = = =
    // 存放特征点数目
    orbex2.N = orbex2.mvKeysUn.size();

    // 计算边界
    orbex2.ComputeImageBounds(src162);
    // 计算网格参数
    orbex2.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(orbex2.mnMaxX-orbex2.mnMinX);
    orbex2.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(orbex2.mnMaxY-orbex2.mnMinY);
    
    // 将特征点分配到图像网格中
    orbex2.AssignFeaturesToGrid();
    
    // 至此已经存在两个包含图像的ORBexractor实例

    // 显示两张已经提取成功的图像

    cv::Mat out1, out2;
    cv::drawKeypoints(src8, orbex.mvKeysUn, out1);
    cv::drawKeypoints(src82, orbex2.mvKeysUn, out2);
    cv::imshow("ORB extract 1", out1);
    cv::imshow("ORB extract 2", out2);
    
    // = = = = = = = = = 寻找匹配关系 = = = = = = = = =
    // 构建匹配的实例
    ORBmatcher matcher(
        0.5,                // 最佳点和次佳特征点评分的比值阈值。此处比较宽松，tracking时一般是0.7
        true                // 检查特征点的方向
        );


    // mvbPrevMatched为参考帧的特征点坐标，初始化存储的是mInitialFrame中的特征点坐标，匹配结束之后就是当前帧的特征点坐标，做一个帧帧之间的迭代操作
    // mvIniMatches 保存参考帧F1中特征点是否匹配上，index保存的是F1对应特征点索引，值保存的是匹配好的F2特征点索引
    // 构建保存匹配关系的容器
    cout << "OKKKKKK" << endl;

    std::vector<int> Matches12;
    // 特征点类型从Keypoints转换为Points2d，内容为参考帧中的内容
    std::vector<cv::Point2f> PrevMatched;
    PrevMatched.resize(orbex.mvKeysUn.size());
    for(int i = 0; i<orbex.mvKeysUn.size(); i++)
        PrevMatched[i] = orbex.mvKeysUn[i].pt;
    
    int nmatches = matcher.SearchForInitialization(
        orbex,                      // 初始化时的参考帧
        orbex2,                     // 初始化时的当前帧
        PrevMatched,                // 在初始化参考帧中提取得到的特征点
        Matches12,                  // 保存匹配的关系
        100                         // 搜索窗口大小
        );
    cout << orbex.mvKeysUn.size() << " Matching Over! " << nmatches << endl;

    // DMatch 作图
    cout << nmatches << " " << Matches12.size() << endl;
    // test 寻找匹配关系中的内容，遍历
    vector<cv::DMatch> dmatch;
    cv::DMatch temp;
    int count = 0;
    for(int i = 0; i < Matches12.size(); ++i)
    {
        if (Matches12[i]!=-1)
        {
            count ++;
            cout << "orbex1 id " << i << " orbex2 id " << Matches12[i] << endl;
            temp.queryIdx = i;
            temp.trainIdx = Matches12[i];
            dmatch.push_back(temp);
        }
    }
    cv::Mat res;
    cout << dmatch.size() << endl << count << endl; 
    cv::drawMatches(src8,orbex.mvKeysUn,src82,orbex2.mvKeysUn, dmatch, res);
    cv::imshow("matches", res);
    cv::imwrite("./res.png", res);

    cv::waitKey();
    
}


