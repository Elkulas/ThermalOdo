#pragma once

#include "algorithm.h"
#include <opencv2/core/core.hpp>
#include <array>


class BitPlaneDescriptor
{
  public:
  // constuctor
  BitPlaneDescriptor(float s0 = 0.5f, float s1 = -1.0);
  // destructor
  ~BitPlaneDescriptor();
  // copy constructor
  BitPlaneDescriptor(const BitPlaneDescriptor& other)
    : rows_(other.rows_), cols_(other.cols_), sigma_ct_(other.sigma_ct_)
    , sigma_bp_(other.sigma_bp_), channels_(other.channels_) {}

  void compute(cv::Mat&);

  private:
    int rows_, cols_;
    float sigma_ct_, sigma_bp_;
    std::array<cv::Mat,8> channels_;
    
    // LBP image
    cv::Mat lbp_;


}; // BitPlaneDescriptor