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

void BitPlaneDescriptor::compute(cv::Mat& I) {
  rows_ = I.rows;
  cols_ = I.cols;
  // Step 1: Get LBP Image
  cv::GaussianBlur(I, I, cv::Size(3,3), sigma_ct_, sigma_ct_);
  getOriginLBPFeature(I, lbp_);

  cv::imshow("lbp", lbp_);
  cv::waitKey();

  // Step 2: Get 8 channel images

    
// cv::Mat out0;
//   ExtractChannel(lbp_, out0, 0, sigma_bp_);
//   std::cout << "Num: 0" << std::endl;
//   cv::imshow("lbp0", out0);
//   cv::waitKey();

  // cv::Mat out;
  // ExtractChannel(lbp_, out, 1, sigma_bp_);
  // std::cout << "TYPE: " << out.type() << std::endl;
  // std::cout << "Num: 1" << std::endl;
  // cv::imshow("lbp2", out);
  // cv::waitKey();

  cv::Mat out2;
  ExtractChannel(lbp_, out2, 2, sigma_bp_);
  std::cout << "Num: 2" << std::endl;
  cv::imshow("lbp4", out2);
  cv::waitKey();

  // cv::Mat out3;
  // ExtractChannel(lbp_, out3, 3, sigma_bp_);
  // std::cout << "Num: 3" << std::endl;
  // cv::imshow("lbp3", out3);
  // cv::waitKey();

// cv::Mat out4;
//   ExtractChannel(lbp_, out4, 4, sigma_bp_);
//   std::cout << "Num: 4" << std::endl;
//   cv::imshow("lbp5", out4);
//   cv::waitKey();

  // cv::Mat out5;
  // ExtractChannel(lbp_, out5, 5, sigma_bp_);
  // std::cout << "Num: 5" << std::endl;
  // cv::imshow("lbp6", out5);
  // cv::waitKey();

  cv::Mat out6;
  ExtractChannel(lbp_, out6, 6, sigma_bp_);
  std::cout << "Num: 6" << std::endl;
  cv::imshow("lbp7", out6);
  cv::waitKey();

  // cv::Mat out7;
  // ExtractChannel(lbp_, out7, 7, sigma_bp_);
  // std::cout << "Num: 7" << std::endl;
  // cv::imshow("lbp8", out7);
  // cv::waitKey();

  // 0 4
  // divide
  // cv::Mat output(out0.size(), out0.type());
  // for(int i = 0; i < rows_; ++i)
  // for(int j = 0; j < cols_; ++j) {
  //   if(j - 3 < 0)
  //     output.at<float>(i, j) = out0.at<float>(i, j);
  //   else{
  //     if((float)out0.at<float>(i, j) - (float)out4.at<float>(i, j - 3) < 0)
  //       output.at<float>(i, j) = 0;
  //     else
  //       output.at<float>(i, j) = (float)out0.at<float>(i, j) - (float)out4.at<float>(i, j - 3);
  //   }
  // }
  // cv::imshow("lbp100", output);
  // cv::waitKey();

  // 2, 6
  cv::Mat output(out2.size(), out2.type());
  for(int i = 0; i < rows_; ++i)
  for(int j = 0; j < cols_; ++j) {
    if(i - 3 < 0)
      output.at<float>(i, j) = out2.at<float>(i, j);
    else{
      if((float)out2.at<float>(i, j) - (float)out6.at<float>(i - 3, j) < 0)
        output.at<float>(i, j) = 0;
      else
        output.at<float>(i, j) = (float)out2.at<float>(i, j) - (float)out6.at<float>(i - 3, j);
    }
  }
  cv::imshow("lbp100", output);
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


