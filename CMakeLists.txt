cmake_minimum_required(VERSION 3.0.0)
project(orbextract VERSION 0.1.0)

set( CMAKE_BUILD_TYPE "Release" )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )

include(CTest)
enable_testing()

set(OpenCV_DIR "/home/jjj/CodeSrc/opencv-4.4.0/build")
find_package(OpenCV 4.0 QUIET)
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

find_package(Eigen3 3.1.0 REQUIRED)
message(STATUS "Eigen3 include path: ${EIGEN3_INCLUDE_DIR}")

find_package(PCL 1.7 REQUIRED)
message(STATUS "pcl include dir ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl VERSION ${PCL_VERSION}")

include_directories( 
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include/orb_extract
    ${PROJECT_SOURCE_DIR}/include/Utils
    ${PROJECT_SOURCE_DIR}/include/localUtils
    
    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/orb_extract/ORBextractor.cc
src/orb_extract/ORBmatcher.cc
src/orb_extract/RtGenerator.cc
src/localUtils/localUtils.cc
src/Utils/projection.cc
src/Utils/feature.cc
src/Utils/algorithm.cc

)

target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
)

# 生成可执行文件
add_executable(orb_extract src/orb_extract.cpp)
target_link_libraries(orb_extract ${PROJECT_NAME})

add_executable(projectRGB src/projectRGB.cpp)
target_link_libraries(projectRGB ${PROJECT_NAME} ${PCL_LIBRARIES} ${EIGEN3_LIBS})

add_executable(projectRGB2 src/projectRGB_Labdata.cpp)
target_link_libraries(projectRGB2 ${PROJECT_NAME} ${PCL_LIBRARIES} ${EIGEN3_LIBS})

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
