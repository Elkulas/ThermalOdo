// C++ 标准库
#include <iostream>
#include <string>
#include <vector>
#include <sstream>
#include <string>
#include <cmath>
#include <fstream>

#include <Eigen/Core>
#include <Eigen/Geometry>

// OpenCV 库
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

// OpenCV 4.0 库
#include <opencv2/ximgproc.hpp>
#include "opencv2/imgcodecs.hpp"
// OpenCV 4.4.0
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

// PCL 库
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// boost
#include <boost/format.hpp>

// time
#include <time.h>


#include "projection.h"
#include "localUtils.h"
#include "feature.h"
#include "algorithm.h"
