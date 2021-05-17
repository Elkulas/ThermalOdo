#include "thermalodo.h"

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

#include "PoseSolve.h"

int main( int argc, char** argv)
{
    string thermal_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561468.864967.png";
    string thermal16_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermalRaw/1578561468.864967.png";
    string thermal2_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561469.052306.png";
    string thermal216_str = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermalRaw/1578561469.052306.png";

    cv::Mat thermal = cv::imread(thermal_str);
    cv::Mat thermal16 = cv::imread(thermal16_str,-1);
    
    cv::Mat thermal2 = cv::imread(thermal2_str);
    cv::Mat thermal216 = cv::imread(thermal216_str,-1);

    // 在rgb上提点
    cv::Mat thermalgrey;
    cv::cvtColor(thermal, thermalgrey, cv::COLOR_BGR2GRAY);
    cv::Mat thermal2grey;
    cv::cvtColor(thermal2, thermal2grey, cv::COLOR_BGR2GRAY);

    cv::Mat grad1 = GetGradImage(thermal16);
    cv::Mat grad2 = GetGradImage(thermal216);

    cv::Mat grad1_8 = Swap16To8(grad1);
    cv::Mat grad2_8 = Swap16To8(grad2);

    vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detectorgftt = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detectorgftt->detect(grad1_8, kp1);

    cv::Mat lbptest1, lbptest2;
    getOriginLBPFeature(grad1_8, lbptest1);
    getOriginLBPFeature(grad2_8, lbptest2);

    vector<cv::Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    
    cv::calcOpticalFlowPyrLK(lbptest1, lbptest2, pt1, pt2, status, error, cv::Size(8, 8));

    SelectMajority(pt1, pt2, status);

    Mat R21,t21,K;
    GetPose(pt1, pt2, status, R21, t21, K, 200);
    return 0;
}