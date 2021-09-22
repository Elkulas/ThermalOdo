/* 
 * Contributor: elkulasjiang@zju.edu.cn
 */

#include "ProcessRelative.h"

#include <opencv2/imgproc/imgproc.hpp>

// constructor
BitPlaneDescriptor::BitPlaneDescriptor(float s0, float s1)
  : rows_(0), cols_(0), sigma_ct_(s0), sigma_bp_(s1) {}

// destructor
BitPlaneDescriptor::~BitPlaneDescriptor() {}

// compute
void BitPlaneDescriptor::compute(cv::Mat& I) {
  rows_ = I.rows;
  cols_ = I.cols;
  // Step 1: Get LBP Image
  cv::GaussianBlur(I, I, cv::Size(3,3), sigma_ct_, sigma_ct_);
  getOriginLBPFeature(I, lbp_);

  cv::imshow("lbp", lbp_);

  cv::waitKey();

  cv::Mat src_proc;
  I.convertTo(src_proc, CV_64FC1);
  cv::Mat spectrum(I.size(), CV_64FC2);

  cv::dft(src_proc, spectrum, cv::DFT_COMPLEX_OUTPUT, 0);
  // shift
  dftShift(spectrum);
 
  cv::Mat out_mag;
  // split channel
  cv::Mat mag_channel[2];
  cv::split(spectrum, mag_channel);
  cv::magnitude(mag_channel[0], mag_channel[1], out_mag);
  cv::normalize(out_mag, out_mag, 0, 255, cv::NORM_MINMAX);
  out_mag.convertTo(out_mag, CV_8UC1);
  cv::imshow("heloojho", out_mag);
  cv::imwrite("/home/jjj/NGCLAB/ThermalOdo/bin/test.png", out_mag);
  cv::waitKey();


  // DFT test


  // Step 2: Get 8 channel images

    
  cv::Mat out0;
  ExtractChannel(lbp_, out0, 0, sigma_bp_);
  std::cout << "Num: 0" << std::endl;
  cv::imshow("lbp0", out0);
  cv::waitKey();

  cv::Mat out;
  ExtractChannel(lbp_, out, 1, sigma_bp_);
  std::cout << "TYPE: " << out.type() << std::endl;
  std::cout << "Num: 1" << std::endl;
  cv::imshow("lbp2", out);
  cv::waitKey();

  cv::Mat out2;
  ExtractChannel(lbp_, out2, 2, sigma_bp_);
  std::cout << "Num: 2" << std::endl;
  cv::imshow("lbp4", out2);
  cv::waitKey();

  cv::Mat out3;
  ExtractChannel(lbp_, out3, 3, sigma_bp_);
  std::cout << "Num: 3" << std::endl;
  cv::imshow("lbp3", out3);
  cv::waitKey();

  cv::Mat out4;
  ExtractChannel(lbp_, out4, 4, sigma_bp_);
  std::cout << "Num: 4" << std::endl;
  cv::imshow("lbp5", out4);
  cv::waitKey();

  cv::Mat out5;
  ExtractChannel(lbp_, out5, 5, sigma_bp_);
  std::cout << "Num: 5" << std::endl;
  cv::imshow("lbp6", out5);
  cv::waitKey();

  cv::Mat out6;
  ExtractChannel(lbp_, out6, 6, sigma_bp_);
  std::cout << "Num: 6" << std::endl;
  cv::imshow("lbp7", out6);
  cv::waitKey();

  cv::Mat out7;
  ExtractChannel(lbp_, out7, 7, sigma_bp_);
  std::cout << "Num: 7" << std::endl;
  cv::imshow("lbp8", out7);
  cv::waitKey();


  // loop
  // for(int b = 0; b < 8; b++) {
  //   cv::Mat out1;
  //   std::cout << "B: " << b << std::endl;
  //   ExtractChannel(lbp_, out1, b, 0.0f);
  //   cv::imshow("lbp3", out1);
  //   cv::waitKey();
  // }



}

void BitPlaneDescriptor::ExtractChannel(const cv::Mat& src, cv::Mat& dst, int bit, float sigma) {
  dst.create(src.size(), cv::DataType<float>::type);
  float scale = float(1.0);
  float bias = float(0.0);

  auto src_ptr = src.ptr<const uint8_t>();
  auto dst_ptr = dst.ptr<float>();

  auto n = rows_*cols_;
  std::cout << "N " << n << std::endl;

  // traverse
  for(int i = 0; i < n; ++i) {
    dst_ptr[i] = scale * ((src_ptr[i] & (1 << bit)) >> bit) - bias;
  }

  // if(sigma > 0.0f) {
  //   cv::GaussianBlur(dst, dst, cv::Size(5,5), sigma, sigma);
  // }
}

void BitPlaneDescriptor::dftShift(cv::Mat& img){
  int cx = img.cols / 2;
  int cy = img.rows / 2;
  cv::Mat q0 = img(cv::Rect(0, 0, cx, cy));
  cv::Mat q1 = img(cv::Rect(cx, 0, cx, cy));
  cv::Mat q2 = img(cv::Rect(0, cy, cx, cy));
  cv::Mat q3 = img(cv::Rect(cx, cy, cx, cy));
  cv::Mat tmp;
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);
  q1.copyTo(tmp);
  q2.copyTo(q1);
  tmp.copyTo(q2);
}



