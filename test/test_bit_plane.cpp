
#include "thermalodo.h"

#include "ProcessRelative.h"
#include <gflags/gflags.h>

DEFINE_string(dir, "", "Input pic directory");

int main( int argc, char* argv[]) {
  std::cout << "Hello, That is Bit Plane Test" << std::endl;

  gflags::ParseCommandLineFlags(&argc, &argv, true);
  cout << "dir = " << FLAGS_dir << endl;

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