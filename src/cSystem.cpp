#include <iostream>
#include "cSystem.h"

using namespace std;

cSystem::cSystem(const std::string& path2calib){
    LoadMCS(path2calib);
}

void cSystem::LoadMCS(const std::string path2calib){
    // Read the multi-camera system configuration file
    string mcs_settings = path2calib + "/MultiCamSys_Calibration.yaml";
    cv::FileStorage mcs_calib_data(mcs_settings, cv::FileStorage::READ);
    nrCams_ = (int)mcs_calib_data["CameraSystem.nrCams"]; // Number of cameras
    
    // Initializing the camera extrinsic matrices
    vector<cv::Matx44d> M_c(nrCams_);
    // Initializing the camera intrinsic matrices
    vector<cv::Mat_<double>> K_c(nrCams_);
    for(int c = 0; c < nrCams_; c++){
        // camera extrinsics are given by the 6 Caley parameters model
        cv::Matx61d tmp;
        for(int p = 1; p < 7; p++){
            string param = "CameraSystem.cam" + to_string(c + 1) + "_" + to_string(p);
            tmp(p - 1) = mcs_calib_data[param];
        }
        M_c[c] = cayley2hom<double>(tmp);
        
        // Intrinsic parameters definition
        string calib_data = path2calib + "/InteriorOrientationFisheye" + to_string(c) + ".yaml";
        cv::FileStorage fSettings(calib_data, cv::FileStorage::READ);
        double pinhole_fx =  fSettings["Camera.pinhole_fx"];
        double pinhole_fy =  fSettings["Camera.pinhole_fy"];
        double pinhole_px =  fSettings["Camera.pinhole_px"];
        double pinhole_py =  fSettings["Camera.pinhole_py"];

        K_c[c] = setPinholeCameraMatrix(pinhole_fx,pinhole_fy,pinhole_px,pinhole_py);
    }
    // Copying variables
    M_c_ = M_c; K_c_ = K_c;
}