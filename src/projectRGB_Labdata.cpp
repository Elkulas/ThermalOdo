// C++ 标准库
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV 库
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

using namespace std;
using namespace ORB_SLAM2;

// 定义点云类型
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 自己的数据集
// Kinect相机内参
// const double kfactor = 1000;
// const double kfx = 536.127551119343;
// const double kfy = 534.274934951329;
// const double kcx = 316.456919698249;
// const double kcy = 248.078476912257; 
const double kfactor = 1000;
const double kfx = 518.0;
const double kfy = 519.0;
const double kcx = 325.5;
const double kcy = 253.5; 

void MapKinectToRGB(vector<Eigen::Vector3d>& keypoints, cv::Mat& rgb, Eigen::Isometry3d& T21, Eigen::Matrix3d& K);
void GeneratePointCloud(cv::Mat& color1, cv::Mat& depth1, cv::Mat& color2, cv::Mat& depth2, Eigen::Isometry3d& T1, Eigen::Isometry3d& T2);
void GeneratePointCloudwithK(cv::Mat& color1, cv::Mat& depth1, cv::Mat& color2, cv::Mat& depth2, Eigen::Isometry3d& T1, Eigen::Isometry3d& T2, Eigen::Matrix3d& K);

int main( int argc, char** argv)
{
    
    // cv::Mat kinectcolor = cv::imread("/home/jjj/NGCLAB/catkin_ws/src/bag_extractor/data/kinectRGB/1578561468.864967.png");
    // cv::Mat depth = cv::imread("/home/jjj/NGCLAB/catkin_ws/src/bag_extractor/data/kinectDepth/1578561468.864967.png", -1);
    // cv::Mat kinectcolor2 = cv::imread("/home/jjj/NGCLAB/catkin_ws/src/bag_extractor/data/kinectRGB/1578561472.365590.png");
    // cv::Mat depth2 = cv::imread("/home/jjj/NGCLAB/catkin_ws/src/bag_extractor/data/kinectDepth/1578561472.365590.png", -1);

    // //1578561468.863539 0.098484 -0.076209 0.063944 0.039908 0.483297 0.005799 -0.874527
    // Eigen::Quaterniond q1(-0.874527, 0.039908, 0.483297, 0.005799);
    // Eigen::Vector3d t1(0.098484, -0.076209, 0.063944);
    // //1578561472.366326 -0.407123 -0.047007 0.043808 0.016689 0.451796 0.001481 -0.891964
    // Eigen::Quaterniond q2(-0.874527, 0.039908, 0.483297, 0.005799);
    // Eigen::Vector3d t2(0.098484, -0.076209, 0.063944);

    Eigen::Matrix3d K;
    K << kfx, 0, kcx, 0, kfy, kcy, 0, 0, 1;
    cout << K .matrix() << endl;

    cv::Mat kinectcolor = cv::imread("/home/jjj/Documents/slambook/ch5/joinMap/color/1.png");
    cv::Mat depth = cv::imread("/home/jjj/Documents/slambook/ch5/joinMap/depth/1.pgm", -1);
    cv::Mat kinectcolor2 = cv::imread("/home/jjj/Documents/slambook/ch5/joinMap/color/2.png");
    cv::Mat depth2 = cv::imread("/home/jjj/Documents/slambook/ch5/joinMap/depth/2.pgm", -1);

    //-0.228993 0.00645704 0.0287837 -0.0004327 -0.113131 -0.0326832 0.993042
    Eigen::Quaterniond q1(0.993042,-0.0004327, -0.113131, -0.0326832 );
    Eigen::Vector3d t1(-0.228993, 0.00645704, 0.0287837);

    //-0.50237 -0.0661803 0.322012 -0.00152174 -0.32441 -0.0783827 0.942662
    Eigen::Quaterniond q2(0.942662,-0.00152174, -0.32441, -0.0783827 );
    Eigen::Vector3d t2(-0.50237, -0.0661803, 0.322012);
    
    Eigen::Isometry3d T1(q1);
    Eigen::Isometry3d T2(q2);
    T1.pretranslate(t1);
    T2.pretranslate(t2);

    Eigen::Isometry3d T21 = T2.inverse() * T1;
    

    cout << T21.matrix() <<endl;

    GeneratePointCloud(kinectcolor, depth, kinectcolor2, depth2, T1, T2);
    GeneratePointCloudwithK(kinectcolor, depth, kinectcolor2, depth2, T1, T2, K);

    // = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = = 
    // Step 1: 对kinect中的RGB图片进行特征点提取 kinectcolor
    // 输出查看特征点
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptor;
    cv::Ptr<cv::ORB> detector = cv::ORB::create(1000,2,8,31,0,2,cv::ORB::ScoreType::HARRIS_SCORE,31,10);
    detector->detect(kinectcolor, keypoints, cv::Mat());

    cout << keypoints.size() <<endl;
    cv::Mat out;
    cv::drawKeypoints(kinectcolor, keypoints, out);
    cv::imshow("show", out);

    // Step 2: 将特征点所在的点还原成三维点
    vector<Eigen::Vector3d> vKinectPoints;
   
    for(auto point:keypoints)
    {
        double d = static_cast<double>(depth.ptr<ushort>((int)(point.pt.y))[(int)(point.pt.x)])/ kfactor;
        
        // 如果深度为零,舍弃
        if(d == 0) continue;

        Eigen::Vector3d p;
        Eigen::Vector3d pixel(double(point.pt.x), double(point.pt.y), double(1.0));

        // cout << pixel.matrix() << endl <<endl;
        p = K.inverse() * (pixel * d);
        // cout << p.matrix() << endl <<endl;
        vKinectPoints.push_back(p);
    }
    cout << vKinectPoints.size() <<endl;
    
    MapKinectToRGB(vKinectPoints, kinectcolor2, T21, K);

    cv::waitKey();
        
    return 0;
}

void  MapKinectToRGB(vector<Eigen::Vector3d>& keypoints, cv::Mat& rgb, Eigen::Isometry3d& T21, Eigen::Matrix3d& K)
{
    // 通过外参转移
    Eigen::Matrix4d T21m4d = T21.matrix();

    vector<Eigen::Vector3d> vRGB; 

    for(auto point:keypoints)
    {
        Eigen::Vector3d pRGB = T21m4d.block<3,3>(0,0) * point + T21m4d.block<3,1>(0,3);
        // cout << point.matrix() <<endl <<endl;
        vRGB.push_back(pRGB);
    }
    // Mapping
    vector<cv::KeyPoint> vKeyRGB;
    
    for(auto point:vRGB)
    {
        // 投影
        //cout << point.matrix() << endl;
        double depthRGB = point(2);
        if(depthRGB <= 0.001) continue;

        Eigen::Vector3d pixelRGB = (1 / depthRGB) * K * point;
        // cout << pixelRGB.matrix() <<endl << endl;
        float u = static_cast<float>(pixelRGB(0));
        float v = static_cast<float>(pixelRGB(1));

        //cout << u << " " << v <<endl;
        // 超出图片尺寸范围的去除
        if(u < 0 || u > rgb.cols || v < 0 || v > rgb.rows) continue;

        // 构建cv::Keypoint的temp实例,将满足要求的点存入
        cv::KeyPoint tempkp;
        tempkp.pt.x = float(u);
        tempkp.pt.y = float(v);

        // 存入存放关键点的容器
        vKeyRGB.push_back(tempkp);
    }
    cv::Mat out12;
    cv::drawKeypoints(rgb, vKeyRGB, out12);
    cv::imshow("keypoints on RGB", out12);
}

void GeneratePointCloud(cv::Mat& color1, cv::Mat& depth1, cv::Mat& color2, cv::Mat& depth2, Eigen::Isometry3d& T1, Eigen::Isometry3d& T2)
{
    PointCloud::Ptr cloud( new PointCloud);

    for(int m = 0; m < depth1.rows;m++)
        for(int n = 0; n < depth1.cols; n++)
        {
            ushort d = depth1.ptr<ushort>(m)[n];
            if(d == 0) continue;
            Eigen::Vector3d point;
            point[2] = double(d) / kfactor;
            point[0] = (n - kcx) * point[2] /kfx;
            point[1] = (m - kcy) * point[2] /kfy;

            Eigen::Vector3d worldpoint = T1 * point;

            PointT p1;
            p1.x = worldpoint[0];
            p1.y = worldpoint[1];
            p1.z = worldpoint[2];
            p1.b = color1.data[m * color1.step + n*color1.channels()];
            p1.g = color1.data[m * color1.step + n*color1.channels()+1];
            p1.r = color1.data[m * color1.step + n*color1.channels()+2];

            cloud->points.push_back(p1);
        }
    
    for(int m = 0; m < depth2.rows;m++)
        for(int n = 0; n < depth2.cols; n++)
        {
            ushort d = depth2.ptr<ushort>(m)[n];
            if(d == 0) continue;
            Eigen::Vector3d point;
            point[2] = double(d) / kfactor;
            point[0] = (n - kcx) * point[2] /kfx;
            point[1] = (m - kcy) * point[2] /kfy;

            Eigen::Vector3d worldpoint = T2 * point;

            PointT p1;
            p1.x = worldpoint[0];
            p1.y = worldpoint[1];
            p1.z = worldpoint[2];
            p1.b = color2.data[m * color2.step + n*color2.channels()];
            p1.g = color2.data[m * color2.step + n*color2.channels()+1];
            p1.r = color2.data[m * color2.step + n*color2.channels()+2];

            cloud->points.push_back(p1);
        }
    
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << cloud->size() <<endl;
    pcl::io::savePCDFile("./map.pcd", *cloud);
}

void GeneratePointCloudwithK(cv::Mat& color1, cv::Mat& depth1, cv::Mat& color2, cv::Mat& depth2, Eigen::Isometry3d& T1, Eigen::Isometry3d& T2, Eigen::Matrix3d& K)
{
    PointCloud::Ptr cloud( new PointCloud);

    for(int m = 0; m < depth1.rows;m++)
        for(int n = 0; n < depth1.cols; n++)
        {
            double d = double(depth1.ptr<ushort>(m)[n]) / kfactor;
            if(d == 0) continue;
            Eigen::Vector3d point;
            Eigen::Vector3d pixel(double(n), double(m), (1.0));
            point = K.inverse() * (pixel*double(d));

            Eigen::Vector3d worldpoint = T1 * point;

            PointT p1;
            p1.x = worldpoint[0];
            p1.y = worldpoint[1];
            p1.z = worldpoint[2];
            p1.b = color1.data[m * color1.step + n*color1.channels()];
            p1.g = color1.data[m * color1.step + n*color1.channels()+1];
            p1.r = color1.data[m * color1.step + n*color1.channels()+2];

            cloud->points.push_back(p1);
        }
    
    for(int m = 0; m < depth2.rows;m++)
        for(int n = 0; n < depth2.cols; n++)
        {
            double d = double(depth2.ptr<ushort>(m)[n])/kfactor;
            if(d == 0) continue;
            Eigen::Vector3d point;
            Eigen::Vector3d pixel(double(n), double(m), (1.0));
            point = K.inverse() * (pixel*double(d));
            Eigen::Vector3d worldpoint = T2 * point;

            PointT p1;
            p1.x = worldpoint[0];
            p1.y = worldpoint[1];
            p1.z = worldpoint[2];
            p1.b = color2.data[m * color2.step + n*color2.channels()];
            p1.g = color2.data[m * color2.step + n*color2.channels()+1];
            p1.r = color2.data[m * color2.step + n*color2.channels()+2];

            cloud->points.push_back(p1);
        }
    
    cloud->is_dense = false;
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cout << cloud->size() <<endl;
    pcl::io::savePCDFile("./mapK.pcd", *cloud);
}

