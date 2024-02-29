#include <iostream>
#include "MultiFrame.h"

using namespace std;


MultiFrame::MultiFrame(const std::vector<cv::Mat>& images,  const cv::Mat &cam_pose, const double &timestamp):
images_(images), timestamp_(timestamp), b_empty(false), world_Tcamera_base_(cam_pose)
{}


