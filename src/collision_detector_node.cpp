#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>

// System Libraries
#include <collision_detector.h>
#include <MultiFrame.h>
#include <cSystem.h>

// ROS libraries
#include "ros/ros.h"
#include "image_transport/image_transport.h"

// OpenCV libraries
#include <opencv2/core/core.hpp>

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

class CollisionNode{
    private:
        // ROS variables
        ros::NodeHandle nh_;
        image_transport::Subscriber image_sub_;
        image_transport::ImageTransport it_;

        // Camera System Definition
        cSystem cam_system_;

        // Camera matrices
        std::vector<cv::Matx61d> M_c_; // Extrinsics Cayley parameters
        std::vector<cv::Matx41d> K_c_; // Intrinsics

    public:
        // Constructor
        CollisionNode(ros::NodeHandle &nh, const string &path2calibrations): 
        nh_(nh), it_(nh), cam_system_(path2calibrations){
        };
        /**
         * Create a new CollisionNode object that sets the camera calibration parameters
         * of the system.
         * @param nh Node handle for the ROS object.
        */
        CollisionNode(ros::NodeHandle &nh): 
        nh_(nh), it_(nh){
            loadParameters();

            //Setting the cam system
            int nrCams;
            nh_.getParam("/CameraSystem_nrCams",nrCams);
            cam_system_.setSystemConfig(nrCams,M_c_,K_c_);
        };
        ~CollisionNode(){};

        /**
         *  Loads the camera configuration parameters from the rosparam server.
        */
        void loadParameters(){
            // Loading the extrinsic cayley parameters for each camera.
            int nrCams;
            nh_.getParam("/CameraSystem_nrCams",nrCams);

            // Initializing the camera extrinsic matrices
            vector<cv::Matx61d> M_c(nrCams);
            // Initializing the camera intrinsic matrices
            vector<cv::Matx41d> K_c(nrCams);
            for(int c = 0 ; c < nrCams; c++){
                // camera extrinsics are given by the 6 Caley parameters model
                cv::Matx61d tmp;
                double value;
                for(int p = 1; p < 7; p++){
                    string param = "/CameraSystem_cam" + to_string(c + 1) + "_" + to_string(p);
                    nh_.getParam(param, value);
                    tmp(p - 1) = value;
                }
                M_c[c] = tmp;

                // Intrinsic parameters definition
                double fx,fy,px,py;
                string sfx = "/Camera_pinhole_fx_cam" + to_string(c+1);
                string sfy = "/Camera_pinhole_fy_cam" + to_string(c+1);
                string spx = "/Camera_pinhole_px_cam" + to_string(c+1);
                string spy = "/Camera_pinhole_py_cam" + to_string(c+1);
                nh_.getParam(sfx,fx);
                nh_.getParam(sfy,fy);
                nh_.getParam(spx,px);
                nh_.getParam(spy,py);
                K_c[c](0,0) = fx;
                K_c[c](1,0) = fy;
                K_c[c](2,0) = px;
                K_c[c](3,0) = py;
            }
            // Saving parameters in class variables
            M_c_ = M_c;
            K_c_ = K_c;
        }

};

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc,&argv,true);
    google::InstallFailureSignalHandler();

    ros::init(argc,argv,"CollisionNode");
    ros::NodeHandle nh("~");

    CollisionNode my_collision_node(nh);


    return 0;
}