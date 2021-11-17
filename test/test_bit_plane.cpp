
#include "thermalodo.h"

#include "ProcessRelative.h"
#include <gflags/gflags.h>
#include <glog/logging.h>

DEFINE_string(dir, "", "Input pic directory");
DEFINE_string(log_dir, "/home/jjj/NGCLAB/ThermalOdo/bin/log", "Log dir");

int main( int argc, char* argv[]) {
  std::cout << "Hello, That is Bit Plane Test" << std::endl;

  google::ParseCommandLineFlags(&argc, &argv, true);
  cout << "Image dir = " << FLAGS_dir << endl;

  google::InitGoogleLogging(argv[0]);

  LOG(INFO) << "Hello, World!";

  cv::Mat img = cv::imread(FLAGS_dir, cv::IMREAD_GRAYSCALE);

  if(img.empty()) {
    printf("failed to read image\n");
    return 0;
  }
  cv::imshow("Hewllo0", img);
  cv::waitKey();

  float sigma_ct = 0.75;
  float sigma_bp = 1.618;

  BitPlaneDescriptor desc(sigma_ct, sigma_bp);

  desc.compute(img);

}