#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "PixelGradient.h"
#include "PixelSelector.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(dir, "", "Input pic directory");

using namespace std;
using namespace cv;

int main(int argc, char* argv[]){

  if(argc < 2){
    std::cout << "Usage: ./test_dso_front --dir= {Pic Directory}" << std::endl;
    return 0;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  Mat img0 = imread(FLAGS_dir, 0);

  // detect points
  PixelGradient *pixelGradent_ = new PixelGradient;
  pixelGradent_->computeGradents(img0);

  PixelSelector::Ptr ps(new PixelSelector(img0.cols, img0.rows, 150, 5));

  float *statusMap = new float[img0.cols * img0.cols];

  // detect
  int num = ps->makeMaps(pixelGradent_, statusMap, 1, false, 2);
  // detect ends
  std::cout << "Get Point number'\t'" << num << std::endl;
  cv::waitKey();

  cv::Mat output;
  img0.copyTo(output);
  cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);

  // traverse points
  for (int y = patternPadding + 1; y < img0.rows - patternPadding - 2; y++)
      for (int x = patternPadding + 1; x < img0.cols - patternPadding - 2; x++) {
        if(statusMap[x + y * img0.cols] != 0){
          cv::circle(output, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 1);
        }
  }

  cv::imshow("output", output);

  cv::waitKey();


  // 测试带mask版本的
  std::cout << "On Test Mask Now " <<std::endl;

  // 生成测试用mask
  cv::Mat mask(img0.rows, img0.cols, CV_8UC1, cv::Scalar(255));
  cv::Mat mask2;
  cv::Mat mask4;

  cv::circle(mask, cv::Point(160, 128), 120, 0, -1);
  cv::circle(mask, cv::Point(480, 128), 90, 0, -1);
  cv::circle(mask, cv::Point(320, 256), 30, 0, -1);
  cv::circle(mask, cv::Point(160, 384), 90, 0, -1);
  cv::circle(mask, cv::Point(480, 384), 90, 0, -1);

  cv::resize(mask, mask2, cv::Size(mask.cols*0.5, mask.rows*0.5));
  cv::resize(mask2, mask4, cv::Size(mask2.cols*0.5, mask2.rows*0.5));


  cv::imshow("mask", mask);
  // cv::imshow("mask2", mask2);
  // cv::imshow("mask4", mask4);
  cv::waitKey();

  cv::Mat gradents_origin = cv::Mat(img0.rows, img0.cols, CV_8UC1);
  cv::Mat gradents_after = cv::Mat(img0.rows, img0.cols, CV_8UC1);

  toCvMat(pixelGradent_->absSquaredGrad[0], gradents_origin);
  cv::imshow("origin gradient", gradents_origin);
  cv::waitKey();

  pixelGradent_->computeGradentsWithMask(img0, mask);
  
  toCvMat(pixelGradent_->absSquaredGrad[0], gradents_after);
  cv::imshow("origin after", gradents_after);
  cv::waitKey();

  // 提点
  float *statusMap2 = new float[img0.cols * img0.cols];

  // detect
  int num2 = ps->makeMaps(pixelGradent_, statusMap2, 1, true, 2);
  // detect ends
  std::cout << "Get Point number'\t'" << num2 << std::endl;

  cv::Mat output2;
  img0.copyTo(output2);
  cv::cvtColor(output2, output2, cv::COLOR_GRAY2RGB);

  // traverse points
  for (int y = patternPadding + 1; y < img0.rows - patternPadding - 2; y++)
      for (int x = patternPadding + 1; x < img0.cols - patternPadding - 2; x++) {
        if(statusMap2[x + y * img0.cols] != 0){
          cv::circle(output, cv::Point(x, y), 3, cv::Scalar(0, 0, 255), 1);
        }
  }

  cv::imshow("output2", output);

  cv::waitKey();

  




  return 0;
}

