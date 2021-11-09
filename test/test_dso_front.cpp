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

//TODO 计算梯度
  PixelGradient *pixelGradent_ = new PixelGradient;
  cv::Mat gradent_0;
//  cv::Mat gradent_1;
  pixelGradent_->computeGradents(img0, gradent_0);
  std::cout << "test" << std::endl;

  imshow("gradent_1", gradent_0);

  cv::waitKey();

//TODO 选择像素
  PixelSelector sel(pixelGradent_->wG[0],pixelGradent_->hG[0]);
  Pnt* points[PYR_LEVELS]; 		//!< 每一层上的点类, 是第一帧提取出来的
  int numPoints[PYR_LEVELS];  	//!< 每一层的点数目

  float *statusMap = new float[pixelGradent_->wG[0] * pixelGradent_->hG[0]];

  bool *statusMapB = new bool[pixelGradent_->wG[0] * pixelGradent_->hG[0]];
  float densities[] = {0.001,0.05,0.15,0.5,1}; // 不同层取得点密度
  sel.currentPotential = 5; // 设置网格大小，3*3大小格
  int *w = &pixelGradent_->wG[0];
  int *h = &pixelGradent_->hG[0];

  for(int lvl=0; lvl<pixelGradent_->pyrLevelsUsed; lvl++)
  {
    int npts;
    // NOTICE: 输出的结果是statusMap,其中第0层和上面的层不一样
    if(lvl == 0) // 第0层提取特征像素
      npts = sel.makeMaps(pixelGradent_, statusMap, densities[lvl] * w[0] * h[0], 1, true, 2);
    else  // 其它层则选出goodpoints
      npts = makePixelStatus(pixelGradent_->dIp[lvl], statusMapB, w[lvl], h[lvl], densities[lvl]*w[lvl]*h[lvl]);

    points[lvl] = new Pnt[npts];

    // set idepth map to initially 1 everywhere.
    int wl = w[lvl], hl = h[lvl]; // 每一层的图像大小, 当前层的情况
    Pnt *pl = points[lvl];  // 每一层上的点
    int nl = 0;
    // 要留出pattern的空间, 2 border
    //[ ***step 3*** ] 在选出的像素中, 添加点信息
    // 这两个循环就是在添加点的信息
    for (int y = patternPadding + 1; y < hl - patternPadding - 2; y++)
      for (int x = patternPadding + 1; x < wl - patternPadding - 2; x++) {
        //if(x==2) printf("y=%d!\n",y);
        // 如果是被选中的像素
        if ((lvl != 0 && statusMapB[x + y * wl]) || (lvl == 0 && statusMap[x + y * wl] != 0)) {
          //assert(patternNum==9);
          pl[nl].u = x ;   //? 加0.1干啥
          pl[nl].v = y ;
          pl[nl].idepth = 1;
          pl[nl].iR = 1;
          pl[nl].isGood = true;
          pl[nl].energy.setZero();
          pl[nl].lastHessian = 0;
          pl[nl].lastHessian_new = 0;
          pl[nl].my_type = (lvl != 0) ? 1 : statusMap[x + y * wl];

          Eigen::Vector3f *cpt = pixelGradent_->dIp[lvl] + x + y * w[lvl]; // 该像素梯度
          float sumGrad2 = 0;
          // 计算pattern内像素梯度和
          for (int idx = 0; idx < patternNum; idx++) {
            int dx = patternP[idx][0]; // pattern 的偏移
            int dy = patternP[idx][1];
            float absgrad = cpt[dx + dy * w[lvl]].tail<2>().squaredNorm();
            sumGrad2 += absgrad;
          }

          //! 外点的阈值与pattern的大小有关, 一个像素是12*12
          //? 这个阈值怎么确定的...
          pl[nl].outlierTH = patternNum * setting_outlierTH;

          nl++;
          assert(nl <= npts);
        }
      }

    numPoints[lvl] = nl; // 点的数目,  去掉了一些边界上的点
    std::cout << "Level " << lvl << " has point " << nl << " points." << std::endl;
  } // 遍历金字塔结束

  // 输出
  cv::Mat output;
  img0.copyTo(output);
  cv::cvtColor(output, output, cv::COLOR_GRAY2RGB);

  // 遍历所有提取出来的lvl=0层上的所有特征点
  for(int i = 0; i < numPoints[0]; i++){

    // draw
    cv::circle(output, cv::Point(points[0][i].u, points[0][i].v), 3, cv::Scalar(0, 255, 0), 1);

  }
  std::cout << "All number is " << numPoints[0] << std::endl;
  cv::imshow("Hello", output);

  cv::waitKey();

  return 0;
}

