// Code for IROS 2022 paper
// Generation code for image is MATLAB version
// First run the MATLAB Code to generate denoise image
// then run this code for the image waiting to test
// TODO: Test ORB, Harris, SIFT, SuperPoint to find the benefit of our image processing 


#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/xfeatures2d.hpp>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "PixelGradient.h"
#include "PixelSelector.h"

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

#include <ctime>

DEFINE_string(dir, "", "Input pic directory");
DEFINE_string(dir2, "", "Input pic2 directory");
DEFINE_int32(num, 100, "Input feature number");

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

Mat detectdso(Mat& img, int num2, Mat& out);

cv::Mat DenoiseNew(cv::Mat& img, double scale = 0.25, bool use_nlm = true, int template_w = 3, int search_w = 11);

cv::Mat Denoise(cv::Mat& img, double scale = 0.25, bool use_nlm = true, int template_w = 3, int search_w = 11);

int main(int argc, char* argv[]){

  if(argc < 2){
    std::cout << "Usage: ./test_image_svd --dir= {Pic Directory} --num= {feature numbers}" << std::endl;
    return 0;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  clock_t st, et;

  Mat origin = imread(FLAGS_dir);

  Mat origin2 = imread(FLAGS_dir2);

  int number = FLAGS_num;


  imshow("origin", origin);
  imshow("origin2", origin2);
  cv::waitKey();

  // 转化灰度以及转化rgb
  Mat origin_gray, origin2_gray, origin_rgb, origin2_rgb;
  cvtColor(origin, origin_gray, COLOR_BGR2GRAY);
  cvtColor(origin2, origin2_gray, COLOR_BGR2GRAY);
  origin.copyTo(origin_rgb);
  origin2.copyTo(origin2_rgb);

  // DSO 提点 
  {
    // Mat out_origin, out_denoise, out_denoise2;
    // origin_rgb.copyTo(out_origin);
    // denoise_rgb.copyTo(out_denoise);

    // Mat output_dso_origin = detectdso(origin_gray, number, out_origin);
    // Mat output_dso_denoise = detectdso(denoise_gray, number, out_denoise);


    // imshow("origin_dso", output_dso_origin);
    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/origin_dso.png", output_dso_origin);
    // imshow("denoise_dso", output_dso_denoise);
    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/denoise_dso.png", output_dso_denoise);
    // cv::waitKey();
  }

  // ORB 提点
  {
    // std::vector<cv::KeyPoint> KeyPoint_origin, KeyPoint_denoise;
    // cv::Ptr<cv::ORB> orb = cv::ORB::create(number);
    // orb->detect(origin_gray, KeyPoint_origin);
    // orb->detect(denoise_gray, KeyPoint_denoise);
    // // 画图
    // Mat out_origin, out_denoise;
    // origin_rgb.copyTo(out_origin);
    // denoise_rgb.copyTo(out_denoise);
    // cv::drawKeypoints(out_origin, KeyPoint_origin, out_origin, cv::Scalar(0, 255, 0));
    // cv::drawKeypoints(out_denoise, KeyPoint_denoise, out_denoise, cv::Scalar(0, 255, 0));
    // cv::imshow("origin_orb", out_origin);
    // cv::imshow("denoise_orb", out_denoise);
    // cv::waitKey();
  }

  // ORBSLAM 提点
  {
    Mat out_origin, out_origin2;
    origin_rgb.copyTo(out_origin);
    origin2_rgb.copyTo(out_origin2);

    int nFeatures = number;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = 15;
    int fMinThFAST = 3;

    ORBextractor orbex1 = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,8);
    ORBextractor orbex2 = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,8);

    orbex1.operator()(origin_gray, cv::Mat(), orbex1.mvKeysUn, orbex1.mDescriptors);
    orbex2.operator()(origin2_gray, cv::Mat(), orbex2.mvKeysUn, orbex2.mDescriptors);
    orbex1.N = orbex1.mvKeysUn.size();
    orbex2.N = orbex2.mvKeysUn.size();


    orbex1.ComputeImageBounds(origin_gray);
    // 计算网格参数
    orbex1.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(orbex1.mnMaxX-orbex1.mnMinX);
    orbex1.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(orbex1.mnMaxY-orbex1.mnMinY);
  
    orbex1.AssignFeaturesToGrid();

    orbex2.ComputeImageBounds(origin2_gray);
    // 计算网格参数
    orbex2.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(orbex2.mnMaxX-orbex2.mnMinX);
    orbex2.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(orbex2.mnMaxY-orbex2.mnMinY);
  
    orbex2.AssignFeaturesToGrid();

    
    
    ORBmatcher matcher(
      0.2,                // 最佳点和次佳特征点评分的比值阈值。此处比较宽松，tracking时一般是0.7
      true                // 检查特征点的方向
    );

    std::vector<int> Matches12;
    // 特征点类型从Keypoints转换为Points2d，内容为参考帧中的内容
    std::vector<cv::Point2f> PrevMatched;
    PrevMatched.resize(orbex1.mvKeysUn.size());
    for(int i = 0; i<orbex1.mvKeysUn.size(); i++)
        PrevMatched[i] = orbex1.mvKeysUn[i].pt;
    
    int nmatches = matcher.SearchForInitialization(
        orbex1,                      // 初始化时的参考帧
        orbex2,                     // 初始化时的当前帧
        PrevMatched,                // 在初始化参考帧中提取得到的特征点
        Matches12,                  // 保存匹配的关系
        100                         // 搜索窗口大小
        );
    
    cout << orbex1.mvKeysUn.size() << " Matching Over! " << nmatches << endl;

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
    cv::drawMatches(out_origin,orbex1.mvKeysUn,out_origin2,orbex2.mvKeysUn, dmatch, res);
    cv::imshow("matches", res);
    // cv::imwrite("./res.png", res);

    cv::waitKey();

    // cv::drawKeypoints(out_origin, orbex1.mvKeysUn, out_origin, cv::Scalar(0, 255, 0));
    // cv::drawKeypoints(out_denoise, orbex2.mvKeysUn, out_denoise, cv::Scalar(0, 255, 0));

    // for(int i = 0; i < orbex1.mvKeysUn.size(); i++){
    //   cv::circle(out_origin, orbex1.mvKeysUn[i].pt, 5, cv::Scalar(0, 255, 0), 1, 16);
    // }
    // for(int i = 0; i < orbex2.mvKeysUn.size(); i++){
    //   cv::circle(out_origin2, orbex2.mvKeysUn[i].pt, 5, cv::Scalar(0, 255, 0), 1, 16);
    // }

    // 匹配 ORB

  }

  // SIFT 提点
  {
    // Mat out_origin, out_denoise;
    // origin_rgb.copyTo(out_origin);
    // denoise_rgb.copyTo(out_denoise);

    // std::vector<cv::KeyPoint> KeyPoint_origin, KeyPoint_denoise;
    // cv::Ptr<cv::SIFT> sift = cv::SIFT::create(number, 3, 0.01f, 3, 1.6);
    // sift->detect(origin_gray, KeyPoint_origin);
    // cout << KeyPoint_origin.size() << endl;
    // sift->detect(denoise_gray, KeyPoint_denoise);
    // // 画图
    // // cv::drawKeypoints(out_origin, KeyPoint_origin, out_origin, cv::Scalar(0, 255, 0));
    // // cv::drawKeypoints(out_denoise, KeyPoint_denoise, out_denoise, cv::Scalar(0, 255, 0));

    // for(int i = 0; i < KeyPoint_origin.size(); i++){
    //   cv::circle(out_origin, KeyPoint_origin[i].pt, 5, cv::Scalar(0, 255, 0), 1, 16);
    // }
    // for(int i = 0; i < KeyPoint_denoise.size(); i++){
    //   cv::circle(out_denoise, KeyPoint_denoise[i].pt, 5, cv::Scalar(0, 255, 0), 1, 16);
    // }

    // cv::imshow("origin_sift", out_origin);
    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/origin_sift.png", out_origin);
    // cv::imshow("denoise_sift", out_denoise);
    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/denoise_sift.png", out_denoise);
    // cv::waitKey();
  } 
  // Harris 提点
  {
    // Mat out_origin, out_denoise;
    // origin_rgb.copyTo(out_origin);
    // denoise_rgb.copyTo(out_denoise);

    // vector<cv::Point2f> n_pts, n_pts2;
    // cv::goodFeaturesToTrack(origin_gray, n_pts, number, 0.001, 2, noArray(), 3, false);

    // cv::goodFeaturesToTrack(denoise_gray, n_pts2, number, 0.005, 2, noArray(), 3, false);

    // for(int i = 0; i < n_pts.size(); i++){
    //   cv::circle(out_origin, n_pts[i], 5, cv::Scalar(0, 255, 0), 1, 16);
    // }

    // for(int i = 0; i < n_pts.size(); i++){
    //   cv::circle(out_denoise, n_pts2[i], 5, cv::Scalar(0, 255, 0), 1, 16);
    // }

    // cv::imshow("origin_harris", out_origin);
    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/origin_harris.jpg", out_origin);
    // cv::imshow("denoise_harris", out_denoise);
    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/denoise_harris.jpg", out_denoise);
    // cv::waitKey();
  }


}






Mat detectdso(Mat& img, int num2, Mat& out)
{
  PixelGradient *pixelGradent_ = new PixelGradient;
  pixelGradent_->computeGradents(img);

  PixelSelector::Ptr ps(new PixelSelector(img.cols, img.rows, num2, 15));

  float *statusMap = new float[img.cols * img.cols];

  // detect
  int num = ps->makeMaps(pixelGradent_, statusMap, 1, false, 2);
  // detect ends
  std::cout << "Get Point number'\t'" << num << std::endl;

  // OUt put
  cv::Mat output;
  img.copyTo(output);
  cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);

  // traverse points
  for (int y = 2 + 1; y < output.rows - 2 - 2; y++)
      for (int x = 2 + 1; x < output.cols - 2 - 2; x++) {
        if(statusMap[x + y * output.cols] != 0){
          cv::circle(out, cv::Point(x, y), 5, cv::Scalar(0, 255, 0), 1, 16);
        }
  }
  
  return out;
}

