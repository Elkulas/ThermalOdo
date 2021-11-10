
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(OpenCV QUIET)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  ${OpenCV_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_dso SHARED
    src/Utils.cpp
    src/PixelGradient.cpp
    src/PixelSelector.cpp
    
)
