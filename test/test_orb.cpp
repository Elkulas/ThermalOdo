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

    cout << FLAGS_dir.append(vpath[0]) <<std::endl;
    

    cv::Mat img0 = imread(FLAGS_dir.append(vpath[0]), -1);

    std::cout <<img0.type() << std::endl;

    cv::Mat test;
    img0.copyTo(test);
    cv::cvtColor(test, test, cv::COLOR_BGR2GRAY);

    // = = = = = = = = = Load Config = = = = = = = = = 
    // fast提取参数 
    int nFeatures = FLAGS_nfeat;
    float fScaleFactor = 1.2;
    int nLevels = 8;
    int fIniThFAST = FLAGS_inith;
    int fMinThFAST = FLAGS_minth;

    // 构造提取器实例
    RtGenerator RtGen = RtGenerator();

    ORBextractor orbex = ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

    orbex.operator()(test, cv::Mat(), orbex.mvKeysUn, orbex.mDescriptors);

    orbex.N = orbex.mvKeysUn.size();

    cout << orbex.N << endl;

    cv::drawKeypoints(img0, orbex.mvKeysUn, img0);
    cv::imshow("ORB extract 1", img0);
    cv::waitKey();

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


