
#include "thermalodo.h"

#include "ProcessRelative.h"


int main( int argc, char** argv) {
  std::cout << "Hello, That is Bit Plane Test" << std::endl;

  cv::Mat img = cv::imread("/media/jjj/shuaibi/NGC_data/desk_xyz/thermal/1578561468.864967.png", cv::IMREAD_GRAYSCALE);

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