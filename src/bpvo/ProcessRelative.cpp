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
  getOriginLBPFeature(I, lbp_);

}


