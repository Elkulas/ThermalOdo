#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include "PixelGradient.h"
#include "PixelSelector.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <ctime>

// #include <opencv2/xfeatures2d.hpp>

// #include "matrix.h"
// #include "mclmcr.h"
#include "mclmcrrt.h"
#include "mclcppclass.h"
#include "libsvd_mean_recompute_denoise_first_eigen_zero.h"


mwArray Mat2mwArray(cv::Mat src){
	// if (src.type() != CV_64FC1);
	// 	src.convertTo(src, CV_64FC1, 1.0, 0);

	mwArray dst(src.rows, src.cols, mxUINT8_CLASS); //
	cv::Mat src_t = src.t();
	dst.SetData(src_t.data, src.rows*src.cols); //
	
	return dst;
}

cv::Mat mwArry2Mat(mwArray src, int rows, int cols){
	if (src.IsEmpty()) //
		return cv::Mat();
 
	cv::Mat dst = cv::Mat::zeros(rows, cols, CV_64FC1);
	for (int j(0); j<rows; ++j)
	{
		double* pdata = dst.ptr<double>(j);
		for (int i(0); i<cols; ++i)
		{
			pdata[i] = src(j + 1, i + 1); /// 元素访问（行号，列号）
      if(i < 10 && j < 5)
      cout << " " << pdata[i] ;
		}
    if(j < 5)
    cout << '\n';
	}
	return dst;
}



DEFINE_string(dir, "", "Input pic directory");

using namespace std;
using namespace cv;

int main(int argc, char* argv[]){

  if(argc < 2){
    std::cout << "Usage: ./test_dso_front --dir= {Pic Directory}" << std::endl;
    return 0;
  }

  google::ParseCommandLineFlags(&argc, &argv, true);

  Mat img0 = imread(FLAGS_dir);

  // svd_mean_recompute_denoise_first_eigen_zero(img0);

  
  clock_t startt0  = clock();

  if(libsvd_mean_recompute_denoise_first_eigen_zeroInitialize())
  {
    cout << "wocao! KEYIDE!" << endl;
  }
  else{
    cout << "heloo" << endl;
  }
  clock_t endt0  = clock();

  cout << "load runtime is "<< (double)(endt0 - startt0) / CLOCKS_PER_SEC << endl;

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

  // cv::Mat test2;
	// test.convertTo(test2, CV_64FC1, 1.0, 0);


  mwArray mtlb_img = Mat2mwArray(test);
  mwArray mtlb_img_cout;

  clock_t startt  = clock();

  svd_mean_recompute_denoise_first_eigen_zero(1, mtlb_img_cout, mtlb_img);
  clock_t endt  = clock();

  cout << "runtime is "<< (double)(endt - startt) / CLOCKS_PER_SEC << endl;


  cv::Mat mtlb_out = mwArry2Mat(mtlb_img_cout, img0.rows, img0.cols);

  cout << mtlb_out.type() << endl;

  cv::Mat outtest;
  clock_t startt2  = clock();

  mtlb_out.convertTo(outtest, CV_8UC1, 255, 0);
  clock_t endt2  = clock();

  cout << "runtime is "<< (double)(endt2 - startt2) / CLOCKS_PER_SEC << endl;

  double minv, maxv;
  cv::Point minid, maxid;


  minMaxLoc(test, &minv, &maxv, &minid, &maxid);
  cout << minv << " " << maxv << " " << maxid << endl;

  imshow("sh", outtest);
  cv::waitKey();

  outtest.copyTo(test);


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

