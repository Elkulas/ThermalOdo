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

  Mat img0 = imread(FLAGS_dir, -1);

  if(img0.type() == 0) {
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
  }

  else {
    std::cout << "Not 8bit!" << std::endl;
    PixelGradient *pixelGradent_ = new PixelGradient;
    pixelGradent_->computeGradentsFor16(img0);

    PixelSelector::Ptr ps(new PixelSelector(img0.cols, img0.rows, 150, 5));

    float *statusMap = new float[img0.cols * img0.cols];

    // detect
    int num = ps->makeMaps(pixelGradent_, statusMap, 1, false, 2);
    // detect ends
    std::cout << "Get Point number'\t'" << num << std::endl;
    cv::waitKey();

    cv::Mat output(img0.size(), CV_8UC1);

    cv::normalize(img0, output, 0, 255, cv::NORM_MINMAX, CV_8UC1);


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
  }

  

  return 0;
}

