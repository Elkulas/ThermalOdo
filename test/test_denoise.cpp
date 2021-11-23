#include "thermalodo.h"

#include "ProcessRelative.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(dir, "", "Input pic directory");
// DEFINE_string(log_dir, "/home/jjj/NGCLAB/ThermalOdo/bin/log2", "Log dir");

int main( int argc, char* argv[]) {
  std::cout << "Hello, That is Denoise Test" << std::endl;

  google::ParseCommandLineFlags(&argc, &argv, true);
  cout << "Image dir = " << FLAGS_dir << endl;

  cv::Mat img = cv::imread(FLAGS_dir, cv::IMREAD_GRAYSCALE);

  if(img.empty()) {
    printf("failed to read image\n");
    return 0;
  }
  cv::imshow("Origin", img);
  cv::waitKey();

}