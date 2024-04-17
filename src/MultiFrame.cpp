#include <iostream>
#include "MultiFrame.h"

using namespace std;

namespace coldetector{
    MultiFrame::MultiFrame(const std::vector<cv::Mat>& images,  const cv::Mat &cam_pose, const double &timestamp, const int &numCams):
    images_(images), timestamp_(timestamp), b_empty(false), world_Tcamera_base_(cam_pose), nrCams(numCams), frame_keypoints_(numCams),
    frame_descriptors_(numCams)
    {}
}

