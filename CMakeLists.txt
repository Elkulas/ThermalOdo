cmake_minimum_required(VERSION 3.0.0)
project(thermalodo VERSION 0.1.0)

set( CMAKE_BUILD_TYPE "Release" )
# set( CMAKE_CXX_STANDARD 17)  
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
# set( CMAKE_CXX_STANDARD_REQUIRED ON)

include(CTest)
enable_testing()

######## packages ########
set(OpenCV_DIR "/home/jjj/CodeSrc/opencv-4.5.4/build")
find_package(OpenCV 4.0 QUIET)

find_package(Eigen3 3.1.0 REQUIRED)

find_package(PCL 1.7 REQUIRED)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system)

message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

message(STATUS "Eigen3 include path: ${EIGEN3_INCLUDE_DIR}")

message(STATUS "pcl include dir ${PCL_INCLUDE_DIRS}")
message(STATUS "pcl VERSION ${PCL_VERSION}")

find_package (gflags REQUIRED)

find_package (glog 0.3.5 REQUIRED)

######## includes ########
include_directories( 
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/orb_extract
    ${PROJECT_SOURCE_DIR}/include/Utils
    ${PROJECT_SOURCE_DIR}/include/localUtils
    ${PROJECT_SOURCE_DIR}/src/bpvo
    ${PROJECT_SOURCE_DIR}/src/dso_front/include

    ${OpenCV_INCLUDE_DIRS} 
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${gflags_INCLUDE_DIR}
)
# MATLAB
set(LIBMWM /usr/local/MATLAB/R2021b/runtime/glnxa64/)
include_directories(
    ${LIBMWM}
)

set(MATLAB_ROOT /usr/local/MATLAB/R2021b/)
include_directories(
    ${MATLAB_ROOT}extern/include/
)

####### subs ######
add_subdirectory(src/dso_front)

######## libs ########

#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_library(${PROJECT_NAME} SHARED
src/orb_extract/ORBextractor.cc
src/orb_extract/ORBmatcher.cc
src/orb_extract/RtGenerator.cc
src/localUtils/localUtils.cc
src/Utils/projection.cc
src/Utils/feature.cc
src/Utils/algorithm.cc
src/Utils/ReadData.cc
src/Utils/PoseSolve.cc
src/Utils/BM3D.cpp
src/bpvo/ProcessRelative.cpp
)


target_link_libraries(${PROJECT_NAME}
${OpenCV_LIBS}
${Boost_LIBRARIES}
)

# 生成可执行文件
add_executable(test_orb_extract test/test_orb_extract.cpp)
target_link_libraries(test_orb_extract ${PROJECT_NAME})

add_executable(test_thermal_association test/test_thermal_association.cpp)
target_link_libraries(test_thermal_association ${PROJECT_NAME} ${PCL_LIBRARIES} ${EIGEN3_LIBS})

add_executable(test_read_img test/test_read_img.cpp)
target_link_libraries(test_read_img ${PROJECT_NAME})

add_executable(test_pose_solve test/test_pose_solve.cpp)
target_link_libraries(test_pose_solve ${PROJECT_NAME})

add_executable(test_bit_plane test/test_bit_plane.cpp)
target_link_libraries(test_bit_plane ${PROJECT_NAME} gflags glog)

add_executable(test_denoise test/test_denoise.cpp)
target_link_libraries(test_denoise ${PROJECT_NAME} gflags glog)

add_executable(test_dft test/test_dft.cpp)
target_link_libraries(test_dft ${PROJECT_NAME})

add_executable(test_image_svd test/test_image_svd.cpp)
target_link_libraries(
    test_image_svd
    ${PROJECT_NAME}
    ${PROJECT_NAME}_dso
    gflags
    glog
)

add_executable(test_dso_front test/test_dso_front.cpp)
target_link_libraries(
    test_dso_front
    ${PROJECT_NAME}
    ${PROJECT_NAME}_dso
    gflags
    glog
)

target_link_libraries(
    test_dso_front
    # /usr/local/MATLAB/R2021b/bin/glnxa64/libmwmclmcr.so
    /usr/local/MATLAB/R2021b/runtime/glnxa64/libmwmclmcrrt.so
    /home/jjj/NGCLab/ThermalOdo/img/libsvd_mean_recompute_denoise_first_eigen_zero.so
)


add_executable(test_orb test/test_orb.cpp)
target_link_libraries(
    test_orb
    ${PROJECT_NAME}
    ${PROJECT_NAME}_dso
    gflags
    glog
)

add_executable(expriment_iros test/expriment_iros.cpp)
target_link_libraries(
    expriment_iros
    ${PROJECT_NAME}
    ${PROJECT_NAME}_dso
    gflags
    glog
)

add_executable(expriment_graduate test/expriment_graduate.cpp)
target_link_libraries(
    expriment_graduate
    ${PROJECT_NAME}
    ${PROJECT_NAME}_dso
    gflags
    glog
)


set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
