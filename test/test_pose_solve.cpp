#include "thermalodo.h"

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

#include "PoseSolve.h"
#include "ReadData.h"

// Thermal相机内参(对于RGB来说)
const float tfx = 1073.717;
const float tfy = 1086.810;
const float tcx = 319.896;
const float tcy = 249.620; 

int main( int argc, char** argv)
{
    // 设置读取的情况
    vector<pair<double, Mat>> vThermalImg;
    string sThermalDataPath = "/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/";
    string sThermalConfigPath = "/media/jjj/shuaibi/NGC_data/desk_xyz/";

    GetImageDataWithNum(vThermalImg, sThermalDataPath, sThermalConfigPath, 5);

    cv::imshow("ji", vThermalImg[0].second);
    cout << vThermalImg[0].second.type()<<endl;
    cv::waitKey();


    cv::Mat grad1 = GetGradImage(vThermalImg[0].second);
    cv::Mat grad2 = GetGradImage(vThermalImg[2].second);

    cv::imshow("hgu", grad1);
    cv::waitKey(0);


    vector<cv::KeyPoint> kp1;
    cv::Ptr<cv::GFTTDetector> detectorgftt = cv::GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detectorgftt->detect(grad1, kp1);

    cv::Mat lbptest1, lbptest2;
    getOriginLBPFeature(vThermalImg[0].second, lbptest1);
    getOriginLBPFeature(vThermalImg[2].second, lbptest2);

    vector<cv::Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    
    cv::calcOpticalFlowPyrLK(lbptest1, lbptest2, pt1, pt2, status, error, cv::Size(8, 8));

    SelectMajority(pt1, pt2, status);

    cv::Mat img2_CV;
    int succ_num = 0;
    cv::cvtColor(vThermalImg[2].second, img2_CV, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            succ_num++;
            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    cout <<"succ tracked " << succ_num << endl;
    cout <<"succ tracked size " << status.size() << endl;
    cv::imshow("hji", img2_CV);
    cv::waitKey();

    Mat R21,t21;

    Mat K(3,3,CV_32FC1);
    cout << "hi"<< K.type()<< endl;
    K = (cv::Mat_<float>(3, 3) << tfx, 0, tcx, 0, tfy, tcy, 0, 0, 1);
    cout << K.type()<< endl;

    GetPose(pt1, pt2, status, R21, t21, K, 200);


    return 0;
}