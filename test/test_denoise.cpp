#include "thermalodo.h"

#include "bm3d.h"
#include <opencv2/xphoto/bm3d_image_denoising.hpp>


#include "ProcessRelative.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(dir, "", "Input pic directory");
// DEFINE_string(log_dir, "/home/jjj/NGCLAB/ThermalOdo/bin/log2", "Log dir");

int main( int argc, char* argv[]) {
  std::cout << "Hello, That is Denoise Test" << std::endl;

  google::ParseCommandLineFlags(&argc, &argv, true);
  cout << "Image dir = " << FLAGS_dir << endl;

  // cv::Mat img0 = cv::imread(FLAGS_dir, cv::IMREAD_GRAYSCALE);
  cv::Mat img0 = cv::imread(FLAGS_dir, -1);
  std::cout << "type" << img0.type() << std::endl;

  double mmin, mmax;
  cv::minMaxIdx(img0, &mmin, &mmax);
  std::cout << "small " << mmin << "big " << mmax << std::endl;



  cv::Mat img;
  img0.copyTo(img);

  if(img.empty()) {
    printf("failed to read image\n");
    return 0;
  }
  cv::imshow("Origin", img);
  cv::waitKey();

  img.convertTo(img, CV_32FC1);

  cv::Mat U, W, V;

  cv::SVD::compute(img, W, U, V, cv::SVD::Flags::MODIFY_A);

  std::cout <<"Origin Size \t" << img.size() << std::endl;
  std::cout <<"U Size \t" << U.size() << std::endl;
  std::cout <<"W Size \t" << W.size() << std::endl;
  std::cout <<"V Size \t" << V.size() << std::endl;

  for(int i = 0; i < 9; i++)
    std::cout << W.ptr<float>(i)[0] << std::endl;

  W.ptr<float>(0)[0] = 0.0;

  cv::Mat w(img.rows, img.rows, CV_32FC1, cv::Scalar(0));
  cv::Mat temp(img.size(), CV_32FC1, cv::Scalar(0));

  for(int i = 0; i < img.rows; i++){
    w.ptr<float>(i)[i] = W.ptr<float>(i)[0];
  }

  temp = U*w*V;

  temp.convertTo(temp, CV_8UC1);

  cv::normalize(temp, temp, 0, 255, cv::NORM_MINMAX, CV_8UC1);

  cv::imshow("out", temp);
  
  cv::waitKey();

  // NLM
  cv::Mat nlm;
  std::cout << img0.type() << std::endl;
  cv::fastNlMeansDenoising(img0, nlm, 3.0, 7, 21);
  cv::imshow("heloo nlm", nlm);
  cv::waitKey();

  // BM3D
  cv::Mat bm3d;
  cv::xphoto::bm3dDenoising(img0, bm3d);
  cv::imshow("heloo bm3d", bm3d);
  cv::waitKey();

  




}