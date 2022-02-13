#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "PixelGradient.h"
#include "PixelSelector.h"

#include <ctime>

DEFINE_string(dir, "", "Input pic directory");

using namespace std;
using namespace cv;


void detectdso(Mat& img);

cv::Mat DenoiseNew(cv::Mat& img, double scale = 0.25, bool use_nlm = true, int template_w = 3, int search_w = 11);

cv::Mat Denoise(cv::Mat& img, double scale = 0.25, bool use_nlm = true, int template_w = 3, int search_w = 11);

int main(int argc, char* argv[]){

  if(argc < 2){
    std::cout << "Usage: ./test_image_svd --dir= {Pic Directory}" << std::endl;
    return 0;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  clock_t st, et;

  Mat img0 = imread(FLAGS_dir);

  imshow("hw", img0);

  cout << "Height " << img0.rows << " Width " << img0.cols << endl;

  cout << "OPENCV SVD = = = = " << endl;

  Mat img_svd;
  img0.copyTo(img_svd);
  cvtColor(img_svd, img_svd, COLOR_BGR2GRAY);

  Mat testm;
  img_svd.copyTo(testm);
  cv::medianBlur(testm, testm, 3);
  detectdso(testm);

  
  st = clock();
  Mat img00 = Denoise(img_svd, 1, true, 7, 21);
  Mat img01 = DenoiseNew(img_svd, 1, true, 7, 21);
  
  et = clock();
  cout << "resize SVD mean runtime is "<< (float)(et - st) / CLOCKS_PER_SEC * 1000 << endl;


  imshow("hisd", img00);
  imshow("hisd2", img01);
  detectdso(img00);

  detectdso(img01);
    

}


void detectdso(Mat& img)
{
  PixelGradient *pixelGradent_ = new PixelGradient;
  pixelGradent_->computeGradents(img);

  PixelSelector::Ptr ps(new PixelSelector(img.cols, img.rows, 50, 5));

  float *statusMap = new float[img.cols * img.cols];

  // detect
  int num = ps->makeMaps(pixelGradent_, statusMap, 1, false, 2);
  // detect ends
  std::cout << "Get Point number'\t'" << num << std::endl;

  // OUt put
  cv::Mat output;
  cv::normalize(img, output, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);

  // traverse points
  for (int y = 2 + 1; y < output.rows - 2 - 2; y++)
      for (int x = 2 + 1; x < output.cols - 2 - 2; x++) {
        if(statusMap[x + y * output.cols] != 0){
          cv::circle(output, cv::Point(x, y), 3, cv::Scalar(0, 255, 0), 1);
        }
  }
  
  cv::imshow("output", output);
  cv::imwrite("/home/jjj/NGCLab/ThermalOdo/temp/detect1.png", output);

  cv::waitKey();


  cv::Mat gradents = cv::Mat(output.rows, output.cols, CV_8UC1);
  toCvMat(pixelGradent_->absSquaredGrad[0], gradents);
  cv::imshow("graedient", gradents);
  cv::imwrite("/home/jjj/NGCLab/ThermalOdo/temp/detect2.png", gradents);

  cv::waitKey();
}

cv::Mat Denoise(cv::Mat& img, double scale, bool use_nlm, int template_w, int search_w){
  
  // type

  cv::Mat src;
  img.copyTo(src);

  if(src.type() == 16)
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);

  src.convertTo(src, CV_32FC1);
  cv::normalize(src, src, 0.0, 1.0, NORM_MINMAX);

  cv::resize(src, src, cv::Size(src.cols * scale, src.rows * scale));

  cv::Mat U, W, V;
  cv::SVD::compute(src, W, U, V);

  cv::Mat w = Mat::zeros(cv::Size(W.rows, W.rows), CV_32FC1);

  W.ptr<float>(0)[0] = 0;

  float mean_val = 0;
  for(int i = 0; i < W.rows; ++i)
    mean_val += W.ptr<float>(i)[0];
  
  mean_val = mean_val / (float)W.rows;

  // 倒序赋值
  for (int i = 0; i < W.rows; ++i)
    w.ptr<float>(i)[i] = mean_val + (float)(W.rows-i);
  
  cv::Mat result = U*w*V;
  
  cv::normalize(result, result, 0.0, 255.0, cv::NormTypes::NORM_MINMAX);

  result.convertTo(result, CV_8UC1);

  if(use_nlm)
    cv::fastNlMeansDenoising(result, result, 3.0F, template_w, search_w);
  // medianBlur(result, result, 3);

  // 放大
  cv::resize(result, result, cv::Size(src.cols / scale, src.rows / scale));

  cv::medianBlur(result, result, 7);

  return result;

}



cv::Mat DenoiseNew(cv::Mat& img, double scale, bool use_nlm, int template_w, int search_w){
  
  // type

  cv::Mat src;
  img.copyTo(src);

  if(src.type() == 16)
    cv::cvtColor(src, src, cv::COLOR_BGR2GRAY);

  src.convertTo(src, CV_32FC1);
  cv::normalize(src, src, 0.0, 1.0, cv::NormTypes::NORM_MINMAX);

  cv::resize(src, src, cv::Size(src.cols * scale, src.rows * scale));

  cv::Mat U, W, V;
  cv::SVD::compute(src, W, U, V);

  cv::Mat w = Mat::zeros(cv::Size(W.rows, W.rows), CV_32FC1);

  W.ptr<float>(0)[0] = 0;

  float mean_val = 0;
  for(int i = 0; i < W.rows; ++i)
    mean_val += W.ptr<float>(i)[0];
  
  mean_val = mean_val / (float)W.rows;

  // 倒序赋值
  for (int i = 0; i < W.rows; ++i)
    w.ptr<float>(i)[i] = mean_val + (float)(W.rows-i);
  
  cv::Mat result = U*w*V;



  result = result + 100.0;

  for(int i = 0; i < result.rows; i++)
  for(int j = 0; j < result.cols; j++){
      if(result.ptr<float>(i)[j] > 200)
      result.ptr<float>(i)[j] = 200;
      else if(result.ptr<float>(i)[j] < 0)
      result.ptr<float>(i)[j] = 0;

      result.ptr<float>(i)[j] = result.ptr<float>(i)[j] * 1.275;
  }

  result.convertTo(result, CV_8UC1);

  if(use_nlm)
    cv::fastNlMeansDenoising(result, result, 3.0F, template_w, search_w);
  // medianBlur(result, result, 3);

  // 放大
  cv::resize(result, result, cv::Size(src.cols / scale, src.rows / scale));

  cv::medianBlur(result, result, 7);

  return result;

}
