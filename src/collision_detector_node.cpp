#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <collision_detector.h>
#include <MultiFrame.h>

// ROS libraries
#include "ros/ros.h"

using namespace std;

// Feature computation and Matching
DEFINE_bool(clahe_processing,true,\
            "Wether or not to pre-process images with CLAHE");
DEFINE_bool(matching_all_vs_all,true,\
            "Try to match features with all the ones recognized in the second iamge");
DEFINE_bool(point_cloud_in_world,true,\
            "Give 3D point cloud locations w.r.t. the world");
DEFINE_bool(consider_chassis,true,\
            "Wether or not to consider the AUV chassis for risñ assesment");
DEFINE_int32(int_epipolar_dst_thr,4,\
            "Threshold to filter outliers in triangulation");

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc,&argv,true);
    google::InstallFailureSignalHandler();

    return 0;
}