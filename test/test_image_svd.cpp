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


cv::Mat Denoise(cv::Mat& img, double scale = 0.25, bool use_nlm = true, int template_w = 3, int search_w = 11){
  
  // type

  cv::Mat src;
  img.copyTo(src);

  if(src.type() == 16)
    cvtColor(src, src, COLOR_BGR2GRAY);

  src.convertTo(src, CV_32FC1);
  cv::normalize(src, src, 0.0, 1.0, NORM_MINMAX);

  resize(src, src, Size(src.cols * scale, src.rows * scale));

  Mat U, W, V;
  cv::SVD::compute(src, W, U, V);

  Mat w = Mat::zeros(Size(W.rows, W.rows), CV_32FC1);

  W.ptr<float>(0)[0] = 0;

  float mean_val = 0;
  for(int i = 0; i < W.rows; ++i)
    mean_val += W.ptr<float>(i)[0];
  
  mean_val = mean_val / (float)W.rows;

  // 倒序赋值
  for (int i = 0; i < W.rows; ++i)
    w.ptr<float>(i)[i] = mean_val + (float)(W.rows-i);
  
  Mat result = U*w*V;
  
  cv::normalize(result, result, 0.0, 255.0, NORM_MINMAX);

  result.convertTo(result, CV_8UC1);

  if(use_nlm)
    fastNlMeansDenoising(result, result, 3.0F, template_w, search_w);
  // medianBlur(result, result, 3);

  // 放大
  resize(result, result, Size(src.cols / scale, src.rows / scale));

  medianBlur(result, result, 3);

  return result;

}


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
  
  Mat test_svd;
  img_svd.convertTo(test_svd, CV_32FC1);
  cv::normalize(test_svd, test_svd, 0.0, 1.0, NORM_MINMAX);

  // 测试普通的
  {
    // Mat U, W, V;
    // st = clock();
    // cv::SVD::compute(test_svd, W, U, V);
    
    
    // Mat w = Mat::zeros(Size(W.rows, W.rows), CV_32FC1);

    // // W.ptr<float>(0)[0] = 0;

    // for (int i = 0; i < W.rows; ++i)
    //   w.ptr<float>(i)[i] = W.ptr<float>(i)[0];
    
    // Mat result = U*w*V;

    // cv::normalize(result, result, 0.0, 255.0, NORM_MINMAX);

    // result.convertTo(result, CV_8UC1);

    // et = clock();
    // cout << "OPENCV SVD runtime is "<< (float)(et - st) / CLOCKS_PER_SEC << endl;

    // imshow("hw2", result);

    // detectdso(result);


    // waitKey();
  }

  // 测试普通的 + mean
  {
    st = clock();
    Mat img00 = Denoise(img_svd, 0.25, true, 3, 7);
    et = clock();
    cout << "resize SVD mean runtime is "<< (float)(et - st) / CLOCKS_PER_SEC * 1000 << endl;
    imshow("hisd", img00);
    detectdso(img00);
    waitKey();
    // Mat U, W, V;
    // st = clock();
    // cv::SVD::compute(test_svd, W, U, V);

    // Mat w = Mat::zeros(Size(W.rows, W.rows), CV_32FC1);

    // W.ptr<float>(0)[0] = 0;

    // // 计算mean
    // float mean_val = 0;
    // for(int i = 0; i < W.rows; ++i)
    //   mean_val += W.ptr<float>(i)[0];
    
    // mean_val = mean_val / (float)W.rows;


    // // 倒序赋值
    // for (int i = 0; i < W.rows; ++i)
    //   w.ptr<float>(i)[i] = mean_val + (float)(W.rows-i);
    
    // Mat result = U*w*V;

    // // 滤波
    // // 中值
    // clock_t st0 = clock();
   
    // cv::normalize(result, result, 0.0, 255.0, NORM_MINMAX);

    // result.convertTo(result, CV_8UC1);

    // fastNlMeansDenoising(result, result, 3.0F, 3, 11);

    // medianBlur(result, result, 3);

    // clock_t et0 = clock();
    // et = clock();

    // cout << "OPENCV SVD mean runtime is "<< (float)(et - st) / CLOCKS_PER_SEC << 
    //         "      Fliter time is "<< (float)(et0 - st0) / CLOCKS_PER_SEC << endl;

    // imshow("mean", result);

    // imwrite("/home/jjj/NGCLab/ThermalOdo/temp/1.png", result);

    // waitKey();

    // detectdso(result);
  }
  
  


  // 测试Umat
  // Umat差别不大

  {
    // UMat U, W, V;
    // st = clock();
    // cv::SVD::compute(test_svd, W, U, V);
    // et = clock();
    // cout << "OPENCV SVD UMAT runtime is "<< (float)(et - st) / CLOCKS_PER_SEC << endl;
    // cout << W.size << " " << U.size << " " << V.size << endl;
  }


  // 测试使用分割STRIPE
  int set_height = img0.rows / 32;

  {
    // Mat split_svd;
    // Mat U1, W1, V1;
    // split_svd = Mat(test_svd, Rect(0, 0, test_svd.cols, set_height));

    // st = clock();
    // cv::SVD::compute(split_svd, W1, U1, V1);
    // et = clock();
    // cout << "SPLIT SVD runtime is "<< (float)(et - st) / CLOCKS_PER_SEC << endl;
    
    // cout << W1.size << " " << U1.size << " " << V1.size << endl;

    // W1.ptr<float>(0)[0] = 0.0;

    // Mat w1 = Mat::zeros(Size(W1.rows, W1.rows), CV_32FC1);



    // for (int i = 0; i < W1.rows; ++i)
    //   w1.ptr<float>(i)[i] = W1.ptr<float>(i)[0];
    
    // Mat result1 = U1*w1*V1;

    // cv::normalize(result1, result1, 0.0, 255.0, NORM_MINMAX);

    // result1.convertTo(result1, CV_8UC1);

    // imshow("hw21", result1);

    // waitKey();
  }
  

  // complete with stripe + UMAT
  {
    // Mat dst_show = Mat(test_svd.size(), CV_8UC1, Scalar(0));
    // for(int i = 0; i < 32; i++)
    // {
    //   Mat src;
    //   src = Mat(test_svd, Rect(0, 0 + i * set_height, test_svd.cols, set_height));

    //   UMat U, W, V;

    //   SVD::compute(src, W, U, V);

    //   int set_dim = min(src.rows, src.cols);

    //   UMat W_;

    //   Mat(set_dim, set_dim, CV_32FC1, Scalar(0)).copyTo(W_);

    //   for (int i = 0; i < set_dim; i++)
    //   {
    //     W_.getMat(ACCESS_READ).at<float>(i, i) = W.getMat(ACCESS_READ).at<float>(i, 0);
    //   }

    //   Mat dst = U.getMat(ACCESS_READ) * W_.getMat(ACCESS_READ) * V.getMat(ACCESS_READ);

    //   cv::normalize(dst, dst, 0.0, 255.0, NORM_MINMAX);

    //   dst.convertTo(dst, CV_8UC1);

    //   dst.copyTo(dst_show(Rect(0, 0 + i * set_height, test_svd.cols, set_height)));
    // }

    // imshow("out3", dst_show);
    // waitKey();
  }
  
  // 压缩图片计算
  {

    // 对testsvd压缩

    float scale = 0.25;

    Mat reshape;

    resize(test_svd, reshape, Size(test_svd.cols * scale, test_svd.rows * scale));

    Mat U, W, V;
    st = clock();
    cv::SVD::compute(reshape, W, U, V);

    Mat w = Mat::zeros(Size(W.rows, W.rows), CV_32FC1);

    W.ptr<float>(0)[0] = 0;

    // 计算mean
    float mean_val = 0;
    for(int i = 0; i < W.rows; ++i)
      mean_val += W.ptr<float>(i)[0];
    
    mean_val = mean_val / (float)W.rows;


    // 倒序赋值
    for (int i = 0; i < W.rows; ++i)
      w.ptr<float>(i)[i] = mean_val + (float)(W.rows-i);
    
    Mat result = U*w*V;

    // 滤波
    // 中值
    clock_t st0 = clock();
   
    cv::normalize(result, result, 0.0, 255.0, NORM_MINMAX);

    result.convertTo(result, CV_8UC1);

    // fastNlMeansDenoising(result, result, 3.0F, 3, 11);

    // medianBlur(result, result, 3);

    clock_t et0 = clock();
    fastNlMeansDenoising(result, result, 3.0F, 3, 11);
    medianBlur(result, result, 3);

    et = clock();

    cout << "resize SVD mean runtime is "<< (float)(et - st) / CLOCKS_PER_SEC << 
            "      Fliter time is "<< (float)(et0 - st0) / CLOCKS_PER_SEC << endl;

    imshow("mean23", result);

    imwrite("/home/jjj/NGCLab/ThermalOdo/temp/2.png", result);

    waitKey();

    // 放大
    resize(result, result, Size(reshape.cols / scale, reshape.rows / scale));

    fastNlMeansDenoising(result, result, 3.0F, 3, 11);

    medianBlur(result, result, 3);

    imshow("resize", result);

    imwrite("/home/jjj/NGCLab/ThermalOdo/temp/3.png", result);

    waitKey();

    detectdso(result);

  }

  // 压缩一半同时压缩到一半
  {

    // 对testsvd压缩

    float scale = 0.25;

    Mat reshape;

    resize(test_svd, reshape, Size(test_svd.cols * scale, test_svd.rows * scale));

    // 剪裁
    Mat cut = Mat(reshape, Rect(0, 0, reshape.cols, reshape.rows * 0.5));

    Mat U, W, V;
    st = clock();
    cv::SVD::compute(cut, W, U, V);

    Mat w = Mat::zeros(Size(W.rows, W.rows), CV_32FC1);

    W.ptr<float>(0)[0] = 0;

    // 计算mean
    float mean_val = 0;
    for(int i = 0; i < W.rows; ++i)
      mean_val += W.ptr<float>(i)[0];
    
    mean_val = mean_val / (float)W.rows;


    // 倒序赋值
    for (int i = 0; i < W.rows; ++i)
      w.ptr<float>(i)[i] = mean_val + (float)(W.rows-i);
    
    Mat result = U*w*V;

    // 滤波
    // 中值
    clock_t st0 = clock();
   
    cv::normalize(result, result, 0.0, 255.0, NORM_MINMAX);

    result.convertTo(result, CV_8UC1);

    // fastNlMeansDenoising(result, result, 3.0F, 3, 11);

    // medianBlur(result, result, 3);

    clock_t et0 = clock();
    fastNlMeansDenoising(result, result, 3.0F, 7, 21);
    medianBlur(result, result, 3);

    et = clock();

    cout << "resize cut SVD mean runtime is "<< (float)(et - st) / CLOCKS_PER_SEC << 
            "      Fliter time is "<< (float)(et0 - st0) / CLOCKS_PER_SEC << endl;

    imshow("cut", result);

    imwrite("/home/jjj/NGCLab/ThermalOdo/temp/4.png", result);


    waitKey();

  }

}


void detectdso(Mat& img)
{
  PixelGradient *pixelGradent_ = new PixelGradient;
  pixelGradent_->computeGradents(img);

  PixelSelector::Ptr ps(new PixelSelector(img.cols, img.rows, 150, 5));

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
  imwrite("/home/jjj/NGCLab/ThermalOdo/temp/detect1.png", output);

  cv::waitKey();


  cv::Mat gradents = cv::Mat(output.rows, output.cols, CV_8UC1);
  toCvMat(pixelGradent_->absSquaredGrad[0], gradents);
  cv::imshow("graedient", gradents);
  imwrite("/home/jjj/NGCLab/ThermalOdo/temp/detect2.png", gradents);

  cv::waitKey();
}
