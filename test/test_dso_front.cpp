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
    std::cout << "Usage: ./PixelProcess --dir= {Pic Directory}" << std::endl;
    return 0;
  }

  gflags::ParseCommandLineFlags(&argc, &argv, true);

  Mat img0 = imread(FLAGS_dir, -1);

  PixelGradient *pixelGradent_ = new PixelGradient;
  cv::Mat gradent_0;
//  cv::Mat gradent_1;
  pixelGradent_->computeGradents(img0, gradent_0);

  imshow("gradent", gradent_0);

  cv::waitKey();

  PixelSelector sel(pixelGradent_->wG[0],pixelGradent_->hG[0]);

  float *statusMap = new float[pixelGradent_->wG[0] * pixelGradent_->hG[0]];

  float den = 0.001;
  float densities[] = {den,den * 2,den * 4 ,0.5,1}; // 不同层取得点密度
  sel.currentPotential = 5; // 设置网格大小，3*3大小格
  int *w = &pixelGradent_->wG[0];
  int *h = &pixelGradent_->hG[0];

  int numpts = sel.makeMaps(pixelGradent_, statusMap, densities[0] * w[0] * h[0], 1, true, 2);

  cv::Mat output;
  img0.copyTo(output);
  cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);

  // traverse points
  for (int y = patternPadding + 1; y < pixelGradent_->hG[0] - patternPadding - 2; y++)
      for (int x = patternPadding + 1; x < pixelGradent_->wG[0] - patternPadding - 2; x++) {
        if(statusMap[x + y * pixelGradent_->wG[0]] != 0){
          cv::circle(output, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 1);
        }
  }

  cv::imshow("Hello23", output);

  cv::waitKey();

  cv::waitKey();

  return 0;
}

