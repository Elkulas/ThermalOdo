#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <sstream>
#include <string>

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "RtGenerator.h"

#include "PixelGradient.h"
#include "PixelSelector.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

using namespace ORB_SLAM2;
using namespace std;


DEFINE_string(dir, "", "Input pic directory");
DEFINE_int32(inith, 50, "Input ini threshold");
DEFINE_int32(minth, 30, "Input ini threshold");
DEFINE_int32(nfeat, 500, "Input ini threshold");


void ComputeFAST(const cv::Mat &image, int &threshold);
std::vector<std::string> get_all(fs::path const & root);

int main(int argc, char* argv[]) {

    if(argc < 2){
        std::cout << "Usage: ./test_dso_front --dir= {Pic Directory}" << std::endl;
        return 0;
    }

    google::ParseCommandLineFlags(&argc, &argv, true);

    fs::path root(FLAGS_dir);
    cout << root.string() << endl;
    std::string ext = ".png";
    std::vector<std::string> vpath;
    vpath = get_all(root);

    // = = = = = = = = = Load Config = = = = = = = = = 
    // fast提取参数 
    int nFeatures = FLAGS_nfeat;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = FLAGS_inith;
    int fMinThFAST = FLAGS_minth;
    // = = = = = = = = = = = = = = = = = = = = = = = = 

    for(int i = 0; i + 1 < vpath.size(); i++){
        
        std::string root1 = FLAGS_dir;
        std::string root2 = FLAGS_dir;
        root1.append(vpath[i]);
        root2.append(vpath[i+1]);

        cv::Mat img1 = cv::imread(root1, -1);
        cv::Mat img2 = cv::imread(root2, -1);

        cv::Mat test1, test2, show1, show2;
        img1.copyTo(test1);
        img1.copyTo(show1);
        img2.copyTo(test2);
        img2.copyTo(show2);

        cv::cvtColor(test1, test1, cv::COLOR_BGR2GRAY);
        cv::cvtColor(test2, test2, cv::COLOR_BGR2GRAY);

        ORBextractor orbex1 = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);
        ORBextractor orbex2 = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

        orbex1.operator()(test1, cv::Mat(), orbex1.mvKeysUn, orbex1.mDescriptors);
        orbex2.operator()(test2, cv::Mat(), orbex2.mvKeysUn, orbex2.mDescriptors);
        orbex1.N = orbex1.mvKeysUn.size();
        orbex2.N = orbex2.mvKeysUn.size();

        cv::drawKeypoints(img1, orbex1.mvKeysUn, img1);
        cv::drawKeypoints(img2, orbex2.mvKeysUn, img2);

        cv::imshow("ORB extract 1", img1);
        cv::imshow("ORB extract 2", img2);

        orbex1.ComputeImageBounds(test1);
        orbex2.ComputeImageBounds(test2);
        orbex1.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(orbex1.mnMaxX - orbex1.mnMinX);
        orbex1.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(orbex1.mnMaxY - orbex1.mnMinY);
        orbex2.mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(orbex2.mnMaxX - orbex2.mnMinX);
        orbex2.mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(orbex2.mnMaxY - orbex2.mnMinY);
        orbex1.AssignFeaturesToGrid();
        orbex2.AssignFeaturesToGrid();

        ORBmatcher matcher(
            0.5,                // 最佳点和次佳特征点评分的比值阈值。此处比较宽松，tracking时一般是0.7
            true                // 检查特征点的方向
        );

        std::vector<int> Matches12;
        // 特征点类型从Keypoints转换为Points2d，内容为参考帧中的内容
        std::vector<cv::Point2f> PrevMatched;
        PrevMatched.resize(orbex1.mvKeysUn.size());
        for(int i = 0; i<orbex1.mvKeysUn.size(); i++)
            PrevMatched[i] = orbex1.mvKeysUn[i].pt;
        
        int nmatches = matcher.SearchForInitialization(
            orbex1,                      // 初始化时的参考帧
            orbex2,                     // 初始化时的当前帧
            PrevMatched,                // 在初始化参考帧中提取得到的特征点
            Matches12,                  // 保存匹配的关系
            100                         // 搜索窗口大小
        );

        cout << nmatches << " " << Matches12.size() << endl;

        vector<cv::DMatch> dmatch;
        cv::DMatch temp;
        int count = 0;
        for(int i = 0; i < Matches12.size(); ++i)
        {
            if (Matches12[i]!=-1)
            {
                count ++;
                // cout << "orbex1 id " << i << " orbex2 id " << Matches12[i] << endl;
                temp.queryIdx = i;
                temp.trainIdx = Matches12[i];
                dmatch.push_back(temp);
            }
        }

        cv::Mat res;
        cv::drawMatches(show1, orbex1.mvKeysUn, show2, orbex2.mvKeysUn, dmatch, res);
        cv::imshow("matches", res);

        cv::waitKey();


    }
        
    return 0;

    
}

std::vector<std::string> get_all(fs::path const & root)
{
    std::vector<std::string> paths;
    std::cout << fs::exists(root) << std::endl;

    if (fs::exists(root) && fs::is_directory(root))
    {
        std::cout << "helloo" << std::endl;
        for (auto const & entry : fs::recursive_directory_iterator(root))
        {
            if (fs::is_regular_file(entry))
                paths.emplace_back(entry.path().filename().string());
        }
    }

    return paths;
}             


