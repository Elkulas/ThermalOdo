// 这个文件用来传递外参之间的特征点
// 使用自己的数据集
// 

#include "thermalodo.h"

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

#include "ReadData.h"

using namespace std;
using namespace ORB_SLAM2;

// 定义点云类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 自己的数据集

// Thermal相机内参(对于RGB来说)
const double tfx = 1073.71767197089;
const double tfy = 1086.81080829320;
const double tcx = 319.896642848093;
const double tcy = 249.620749219573; 
// const double tfx = 437.38861083256637;
// const double tfy = 437.29475745770907;
// const double tcx = 323.5284494924228;
// const double tcy = 256.36315482047905; 

// 时间变量
clock_t tstart,tend;

int main( int argc, char** argv)
{
    string thermal_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561468.864967.png";
    string thermal16_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermalRaw/1578561468.864967.png";
    string thermal2_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561469.052306.png";
    string thermal216_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermalRaw/1578561469.052306.png";

    cv::Mat thermal = cv::imread(thermal_str);
    cv::Mat thermal16 = cv::imread(thermal16_str,-1);
    
    cv::Mat thermal2 = cv::imread(thermal2_str);
    cv::Mat thermal216 = cv::imread(thermal216_str,-1);

    // 在rgb上提点
    cv::Mat thermalgrey;
    cv::cvtColor(thermal, thermalgrey, cv::COLOR_BGR2GRAY);
    cv::Mat thermal2grey;
    cv::cvtColor(thermal2, thermal2grey, cv::COLOR_BGR2GRAY);
    //cv::GaussianBlur(thermalgrey, thermalgrey, cv::Size(5,5), 1);

    // =======================================================================================================================
    // 测试LBP
    // =======================================================================================================================
    // 测试LBP测试图像
    // string LBP1str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561489.150462.png";
    // string LBP2str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561489.430732.png";
    // string LBP1str16 = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermalRaw/1578561489.150462.png";
    // cv::Mat LBP1 = cv::imread(LBP1str, 0);
    // cv::Mat LBP2 = cv::imread(LBP2str, 0);
    // cv::Mat LBP116 = cv::imread(LBP1str16, -1);


    // cv::Mat dst1, dst2, dst3, dst4, dst5;
    // getOriginLBPFeature(LBP1, dst1);
    // getOriginLBPFeature(LBP2, dst2);
    // getOriginLBPFeature16(LBP116, dst3);
    // getResidual(dst1, dst3, dst4);

    // getCircularLBPFeatureOptimization(LBP1, dst5, 20, 8);
    

    // cv::imshow("ORIGIN1", LBP1);
    // cv::imshow("ORIGIN2", LBP2);
    // cv::imshow("LBP1 TEST", dst1);
    // cv::imshow("LBP2 TEST", dst2);
    // cv::imshow("LBP3 TEST", dst3);
    // cv::imshow("Circulr LBP5 TEST", dst5);
    // cv::imshow("LBP4 RESIDULA", dst4);
    // cv::waitKey();
    // =======================================================================================================================
    
    
    // =======================================================================================================================
    tstart = clock();

    cv::Mat grad1 = GetGradImage(thermal16);
    cv::Mat grad2 = GetGradImage(thermal216);
    tend = clock();
    cout<< "Time Count: GetGradImage cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
        << "ms" << endl;

    cv::Mat grad1_8 = Swap16To8(grad1);
    cv::Mat grad2_8 = Swap16To8(grad2);

    cout << thermal16.step  << endl;

    // boost::format outfmt("/home/jjj/NGCLAB/dataset/rain+ppl/img_8_grad_byadd/%d-16.png");
    // boost::format fmt("/home/jjj/NGCLAB/dataset/rain+ppl/img_16/%d-16.png");
    // for (int i = 50; i < 4600; i++)
    // {
    //     cv::Mat image = cv::imread((fmt % i).str(), -1);
    //     cv::Mat gd = GetGradImageByAdd(image);
    //     cv::imwrite((outfmt % i).str(), Swap16To8(gd));
    //     cout << i <<endl;
    // }

    // test3 测试opencv光流对特征点之间 的tracking
    // =======================================================================================================================
    // 对于梯度图的tracking
    // =======================================================================================================================
    // 梯度图的提点

    tstart = clock();

    vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detectorgftt = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detectorgftt->detect(grad1_8, kp1);

    tend = clock();
    cout<< "Time Count: GFTT Feature detect cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
        << "ms" << endl;
    
    // 自己的一个提取代码
    // int nFeatures = 500;
    // float fScaleFactor = 1.2;
    // int nLevels = 8;
    // int fIniThFAST = 20;
    // int fMinThFAST = 15;
    
    // ORBextractor orbex = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    // tstart = clock();   
    // orbex.operator()(grad1, cv::Mat(), orbex.mvKeysUn, orbex.mDescriptors);

    // vector<cv::KeyPoint> kp11 = orbex.mvKeysUn;

    // tend = clock();
    // cout<< "Time Count: ORB Feature detect cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
    //     << "ms" << endl;

    // cv::Mat out43;
    // cv::drawKeypoints(grad1_8, kp11, out43);
    // cv::imshow("showORB", out43);

    // cv::waitKey();
    // =======================================================================================================================
    cv::Mat out23;
    cv::drawKeypoints(grad1_8, kp1, out23);
    cv::imshow("show", out23);

    cv::waitKey();
    // =======================================================================================================================

    // 原始图片的提点
    // vector<cv::KeyPoint> kporig;
    // detectorgftt->detect(thermal, kporig);
    // cv::Mat out33;
    // cv::drawKeypoints(thermal, kporig, out33);
    // cv::imshow("sjow", out33);
    // cv::waitKey();

    // =======================================================================================================================
    // Grad梯度图测试光流追踪
    // 追踪代码一致,图像换成Grad图像,使用OpenCV进行track
    // =======================================================================================================================
    // vector<cv::Point2f> pt1, pt2;
    // for (auto &kp: kp1) pt1.push_back(kp.pt);
    // vector<uchar> status;
    // vector<float> error;

    // tstart = clock();
    // cv::calcOpticalFlowPyrLK(grad1_8, grad2_8, pt1, pt2, status, error, cv::Size(8, 8));
    // cout << "pt1 size" << pt1.size() << " pt2 size " << pt2.size() << endl;
    // tend = clock();
    // cout<< "Time Count: OpenCV calcOpticalFlowPyrLK for LBP cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
    //     << "ms" << endl;
    
    // SelectMajority(pt1, pt2, status);

    // cv::Mat img2_CV;
    // int succ_num = 0;
    // cv::cvtColor(grad2_8, img2_CV, cv::COLOR_GRAY2BGR);
    // for (int i = 0; i < pt2.size(); i++) {
    //     if (status[i]) {
    //         succ_num++;
    //         cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
    //         cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
    //     }
    // }
    // cout <<"succ tracked " << succ_num << endl;
    // cout <<"succ tracked size " << status.size() << endl;


    // cv::imshow("tracked by opencv", img2_CV);
    // cv::waitKey(0);

    // =======================================================================================================================
    // 测试LBP图像上的追踪情况
    // 追踪代码一致,图像换成LBP图像,使用OpenCV进行track
    // =======================================================================================================================
    // tstart = clock();

    // cv::Mat lbptest1, lbptest2;
    // getOriginLBPFeature(thermalgrey, lbptest1);
    // getOriginLBPFeature(thermal2grey, lbptest2);
    
    // tend = clock();
    // cout<< "Time Count: getOriginLBPFeature cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
    //     << "ms" << endl;

    // cv::imshow("lbpshow", lbptest1);
    // cv::imshow("lbpshow2", lbptest2);
    // cv::waitKey();

    // vector<cv::Point2f> pt1, pt2;
    // for (auto &kp: kp1) pt1.push_back(kp.pt);
    // vector<uchar> status;
    // vector<float> error;

    // tstart = clock();
    // cv::calcOpticalFlowPyrLK(lbptest1, lbptest2, pt1, pt2, status, error, cv::Size(8, 8));
    // cout << "pt1 size" << pt1.size() << " pt2 size " << pt2.size() << endl;

    // tend = clock();
    // cout<< "Time Count: OpenCV calcOpticalFlowPyrLK for LBP cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
    //     << "ms" << endl;
    
    // tstart = clock();
    // SelectMajority(pt1, pt2, status);
    // tend = clock();
    // cout<< "Time Count: SelectMajority cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
    //     << "ms" << endl;

    // cv::Mat img2_CV;
    // int succ_num = 0;
    // cv::cvtColor(grad2_8, img2_CV, cv::COLOR_GRAY2BGR);
    // for (int i = 0; i < pt2.size(); i++) {
    //     if (status[i]) {
    //         succ_num++;
    //         cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
    //         cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
    //     }
    // }
    // cout <<"succ tracked " << succ_num << endl;
    // cout <<"succ tracked size " << status.size() << endl;


    // cv::imshow("tracked by opencv", img2_CV);
    // cv::waitKey(0);
    // =======================================================================================================================

    // =======================================================================================================================
    // 测试Grad + LBP图像上的追踪情况
    // 追踪代码一致,图像做Grad+LBP变换,使用OpenCV进行track
    // =======================================================================================================================
    tstart = clock();

    cv::Mat lbptest1, lbptest2;
    getOriginLBPFeature(grad1_8, lbptest1);
    getOriginLBPFeature(grad2_8, lbptest2);
    
    tend = clock();
    cout<< "Time Count: getOriginLBPFeature cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
        << "ms" << endl;

    cv::imshow("lbpshow", lbptest1);
    cv::imshow("lbpshow2", lbptest2);
    cv::waitKey();

    vector<cv::Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;

    tstart = clock();
    cv::calcOpticalFlowPyrLK(lbptest1, lbptest2, pt1, pt2, status, error, cv::Size(8, 8));
    cout << "pt1 size" << pt1.size() << " pt2 size " << pt2.size() << endl;

    tend = clock();
    cout<< "Time Count: OpenCV calcOpticalFlowPyrLK for LBP cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
        << "ms" << endl;
    
    tstart = clock();
    SelectMajority(pt1, pt2, status);
    tend = clock();
    cout<< "Time Count: SelectMajority cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
        << "ms" << endl;

    cv::Mat img2_CV;
    int succ_num = 0;
    cv::cvtColor(grad2_8, img2_CV, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            succ_num++;
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    cout <<"succ tracked " << succ_num << endl;
    cout <<"succ tracked size " << status.size() << endl;


    cv::imshow("tracked by opencv", img2_CV);
    cv::waitKey(0);
    // =======================================================================================================================


    // test4 测试自己的光流,使用不同的提取器
    vector<cv::KeyPoint> kp2_multi;
    vector<bool> success_multi;

    tend = clock();

    OpticalFlowMultiLevel(grad1, grad2, kp1, kp2_multi, success_multi, true);
    tend = clock();
    cout<< "Time Count: OpticalFlowMultiLevel for GRAD cost "<< (double)(tend-tstart)/CLOCKS_PER_SEC * 1000 
        << "ms" << endl;

    // Test
    // 将keypoint变成pt
    vector<cv::Point2f> mpt2;
    for (auto &kp: kp2_multi) mpt2.push_back(kp.pt);

    SelectMajorityBool(pt1, mpt2, success_multi);

    Mat img2_multi;
    int multi_sum = 0;
    cv::cvtColor(grad2_8, img2_multi, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < kp2_multi.size(); i++) {
        if (success_multi[i]) {
            multi_sum++;
            cv::circle(img2_multi, kp2_multi[i].pt, 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_multi, kp1[i].pt, kp2_multi[i].pt, cv::Scalar(0, 250, 0));
        }
    }
    cout << "succ tracked " << multi_sum << endl;
    cv::imshow("tracked multi level", img2_multi);
    cv::waitKey();

    // ======================================================================================================================
        
    return 0;
}


