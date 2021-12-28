#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "PixelGradient.h"
#include "PixelSelector.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <opencv2/xfeatures2d.hpp>

DEFINE_string(dir, "", "Input pic directory");

using namespace std;
using namespace cv;

int main(int argc, char* argv[]){

  if(argc < 2){
    std::cout << "Usage: ./test_dso_front --dir= {Pic Directory}" << std::endl;
    return 0;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  Mat img0 = imread(FLAGS_dir, -1);

  std::cout <<img0.type() << std::endl;

  // cv::fastNlMeansDenoising(img0, img0);
  // cv::medianBlur(img0, img0, 3);

  // test： For range detect
  // double minv, maxv;
  // cv::minMaxIdx(img0, &minv, &maxv);
  // std::cout << minv << '\t' << maxv <<std::endl;

  // // cv::Mat test;
  // // img0.copyTo(test);

  // cv::Mat test(img0.size(), CV_8UC1);
  // double minrange = minv;
  // double maxrange = 4000;

  // for (int j = 0; j < img0.rows; ++j) {
  //   for (int i = 0; i < img0.cols; ++i) {

  //     if(img0.at<ushort>(j, i) < maxrange)
  //       (test.at<uchar>(j, i)) = ((img0.at<ushort>(j, i) - minrange) / (maxrange - minrange)) * 255.0;
  //     else
  //       test.at<uchar>(j, i) = 255;
  //   }
  // }
  // cv::imshow("shoe", test);

  // cv::imwrite("/home/jjj/NGCLab/ThermalOdo/6.png", test);
  // cv::waitKey();

  cv::Mat test;
  img0.copyTo(test);
  cv::cvtColor(test, test, cv::COLOR_BGR2GRAY);
  


  // detect points
  PixelGradient *pixelGradent_ = new PixelGradient;
  pixelGradent_->computeGradents(test);

  PixelSelector::Ptr ps(new PixelSelector(test.cols, test.rows, 150, 5));

  float *statusMap = new float[test.cols * test.cols];

  // detect
  int num = ps->makeMaps(pixelGradent_, statusMap, 1, false, 2);
  // detect ends
  std::cout << "Get Point number'\t'" << num << std::endl;

  // OUt put
  cv::Mat output;
  cv::normalize(test, output, 0, 255, cv::NORM_MINMAX, CV_8UC1);
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


  cv::Mat gradents = cv::Mat(img0.rows, img0.cols, CV_8UC1);
  toCvMat(pixelGradent_->absSquaredGrad[0], gradents);
  cv::imshow("graedient", gradents);

  cv::waitKey();

  return 0;
}

