#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace cv;

/**
 * @brief 单层光流
 * @param [in] img1 第一张图像
 * @param [in] img2 第二张图像
 * @param [in] kp1 第一张图像中特征点
 * @param [in|out] kp2 第二张图像中特征点,如果为空,使用第一张图像中作为初始估计
 * @param [out] success 如果一个特征点成功追踪,那么便设置为真
 * @param [in] 是否使用反向光流
 */
void OpticalFlowSingleLevel(
        const cv::Mat &img1,
        const cv::Mat &img2,
        const vector<cv::KeyPoint> &kp1,
        vector<cv::KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * @brief 多层光流,金字塔缩放系数为2,参数设置在函数中
 * @param [in] img1 第一张中的金字塔图
 * @param [in] img2 第二张中的金字塔图
 * @param [in] kp1 第一张图像中特征点
 * @param [out] kp2 第二张图像中特征点
 * @param [out] success 如果一个特征点成功追踪,那么便设置为真
 * @param [in] inverse 是否反向
 */
void OpticalFlowMultiLevel(
        const Mat &img1,
        const Mat &img2,
        const vector<KeyPoint> &kp1,
        vector<KeyPoint> &kp2,
        vector<bool> &success,
        bool inverse = false
);

/**
 * @brief 双线性差值
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    if(img.type() == 0)
    {
        uchar *data = &img.data[int(y) * img.step + int(x)];
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float(
                (1 - xx) * (1 - yy) * data[0] +
                xx * (1 - yy) * data[1] +
                (1 - xx) * yy * data[img.step] +
                xx * yy * data[img.step + 1]
        ); 
    }
    else if(img.type() == 2)
    {
        ushort *data = reinterpret_cast<ushort*>(img.data);
        int ix = int(x);
        int iy = int(y);
        int step = img.step1();
        float xx = x - floor(x);
        float yy = y - floor(y);
        return float(
                (1 - xx) * (1 - yy) * data[iy * step + ix] +
                xx * (1 - yy) * data[iy * step + ix + 1] +
                (1 - xx) * yy * data[(iy+1) * step + ix] +
                xx * yy * data[(iy+1) * step + ix + 1]
        );
    }

}

/**
 * @brief 双线性差值16bit
 * @param img
 * @param x
 * @param y
 * @return
 */
inline float GetPixelValue16(const cv::Mat &img, float x, float y) {

    // uchar *data = &img.data[int(y) * img.step + int(x)];
    ushort *data = reinterpret_cast<ushort*>(img.data);
    int ix = int(x);
    int iy = int(y);
    int step = img.step1();
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[iy * step + ix] +
            xx * (1 - yy) * data[iy * step + ix + 1] +
            (1 - xx) * yy * data[(iy+1) * step + ix] +
            xx * yy * data[(iy+1) * step + ix + 1]
    );
}

void getOriginLBPFeature(cv::Mat& _src, cv::Mat& _dst);

void getOriginLBPFeature16(cv::Mat& _src, cv::Mat& _dst);

void getResidual(cv::Mat& img1, cv::Mat& img2, cv::Mat& dst);

void getCircularLBPFeatureOptimization(cv::Mat& _src, cv::Mat& _dst, int radius, int neighbors);

void SelectMajority(vector<cv::Point2f>& pt1, vector<cv::Point2f>& pt2, vector<uchar>& status);

void ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3);

void SelectMajorityBool(vector<cv::Point2f>& pt1, vector<cv::Point2f>& pt2, vector<bool>& status);