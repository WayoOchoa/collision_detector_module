#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <collision_detector.h>

using namespace std;

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc,&argv,true);
    google::InstallFailureSignalHandler();

    return 0;
}