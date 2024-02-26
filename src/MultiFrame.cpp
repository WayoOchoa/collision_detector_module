#include <iostream>
#include "MultiFrame.h"

using namespace std;


MultiFrame::MultiFrame(const std::vector<cv::Mat>& images, const double &timestamp):
images_(images), timestamp_(timestamp)
{
    //TODO: SET CAMERA SYSTEM K AND [R|T] + NUM CAMS
}


