#include <iostream>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <thread>

// System Libraries
#include <collision_detector.h>
#include <MultiFrame.h>
#include <cSystem.h>

// ROS libraries
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include <cv_bridge/cv_bridge.h>
#include "collision_detection_module/TransferData.h"
#include "nav_msgs/Odometry.h"
#include "cola2_msgs/NavSts.h"

// OpenCV libraries
#include <opencv2/core/core.hpp>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// PointCloud type definition
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;

// Feature computation and Matching parameters
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
DEFINE_string(detector_type,"SIFT",\
            "Set the type of feature detector to be used.");
DEFINE_int32(number_of_features,100,\
            "Number of features to compute in the feature detection process.");
DEFINE_int32(min_Hessian,200,\
            "Threshold for hessian keypoint detector used in SURF.");
DEFINE_int32(nOctaves,4,\
            "Number of pyramid octaves the keypoint detector will use.");
DEFINE_int32(nOctaveLayers,4,\
            "The number of layers in each octave.");
DEFINE_double(contrastThreshold,0.04,\
            "The contrast threshold used to filter out weak features in semi-uniform (low-contrast) regions.\
            The larger the threshold, the less features are produced by the detector.");
DEFINE_double(edgeThreshold,10, "The threshold used to filter out edge-like features.");
DEFINE_double(sigma,1.6, "The sigma of the Gaussian applied to the input image at the octave #0.");

class CollisionNode{
    private:
        // ROS variables
        ros::NodeHandle nh_;
        /**
         * Defining all the subscribers for reading the camera images
        */
        image_transport::SubscriberFilter image_sub_cam0_,image_sub_cam1_,image_sub_cam2_,
                        image_sub_cam3_,image_sub_cam4_,image_sub_cam5_;
        /**
         * Filters for synchronizing the image messages
        */
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, 
                sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,nav_msgs::Odometry, cola2_msgs::NavSts> ApproximatePolicy;
        typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
        boost::shared_ptr<ApproximateSync> m_approximateSync;
        image_transport::ImageTransport it_;
        /**
         * Girona500/1000 Navigation subscribers
        */
        message_filters::Subscriber<nav_msgs::Odometry> odometry_sub_;
        message_filters::Subscriber<cola2_msgs::NavSts> navigation_sub_;
        /**
         * Callable service to transfer data to the collision detection thread
        */
        ros::ServiceServer transfer_frame_data_service_;
        /**
         * Point Cloud publisher
        */
        ros::Publisher pc_pub_;

        // Camera System Definition
        cSystem cam_system_;
        // Camera origin position in the robot chassis
        cv::Mat robot_base_Tcamera_base_;
        // Variable that contains the images
        MultiFrame current_frame_;

        // Camera matrices
        std::vector<cv::Matx61d> M_c_; // Extrinsics Cayley parameters
        std::vector<cv::Matx41d> K_c_; // Intrinsics

        // Collision detection thread
        coldetector::CollisionDetector* coldetector;
        std::thread* CollisionDetection;

        // mutex variables
        std::mutex mFrameData;

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

            // Publishers initialization
            pc_pub_ = nh_.advertise<PointCloud>("local_point_cloud",2);

            // Starting collision detection thread
            coldetector = new coldetector::CollisionDetector(&cam_system_,&pc_pub_);
            CollisionDetection = new thread(&coldetector::CollisionDetector::Run, coldetector);

            // Initializing service for data transfer
            transfer_frame_data_service_ = nh_.advertiseService("transfer_frame_data",&CollisionNode::TransferDataCallback, this);

            // Initializing node subscribers
            // Image subsribers
            int m_queueSize = 1;
            int policy_queuesize = 2;
            image_sub_cam0_.subscribe(it_,"/ladybug1/image_color",m_queueSize);
            image_sub_cam1_.subscribe(it_,"/ladybug2/image_color",m_queueSize);
            image_sub_cam2_.subscribe(it_,"/ladybug3/image_color",m_queueSize);
            image_sub_cam3_.subscribe(it_,"/ladybug4/image_color",m_queueSize);
            image_sub_cam4_.subscribe(it_,"/ladybug5/image_color",m_queueSize);
            image_sub_cam5_.subscribe(it_,"/ladybug6/image_color",m_queueSize);
            // Girona Navigation subscribers
            odometry_sub_.subscribe(nh_,"/navigator/odometry",m_queueSize); // TODO: ASK why navigator/odometry is not working. is the dynamics one the same?
            navigation_sub_.subscribe(nh_,"/navigator/navigation",m_queueSize);
            // Sync policy for the camera messages
            m_approximateSync.reset(new ApproximateSync( ApproximatePolicy(policy_queuesize), image_sub_cam0_, image_sub_cam1_,
                image_sub_cam2_, image_sub_cam3_, image_sub_cam4_, image_sub_cam5_, odometry_sub_, navigation_sub_));
            m_approximateSync->registerCallback(boost::bind(&CollisionNode::SyncImageCallback, this, _1, _2, _3, _4, _5, _6, _7, _8));
        };
        ~CollisionNode(){};

        /**
         * Reads the images coming from the simulator.
         * @brief Callback function for the image subscriber.
         * @param msgx Images sensor_msg received when subscribing to the topic.
        */
        void SyncImageCallback(const sensor_msgs::ImageConstPtr& msg0,const sensor_msgs::ImageConstPtr& msg1, const sensor_msgs::ImageConstPtr& msg2,
            const sensor_msgs::ImageConstPtr& msg3,const sensor_msgs::ImageConstPtr& msg4,const sensor_msgs::ImageConstPtr& msg5,
            const nav_msgs::OdometryConstPtr& odometry_msg, const cola2_msgs::NavStsConstPtr& navigation_msg){
            bool b_frame_incomplete = false;
            // Defining the current pose of the robot
            double roll = navigation_msg->orientation.roll, pitch = navigation_msg->orientation.pitch, yaw = navigation_msg->orientation.yaw;
            cv::Mat rotation_roll_matrix = (cv::Mat_<double>(4,4) << 1,0,0,0,
                    0,cos(roll),-sin(roll),0,
                    0,sin(roll),cos(roll),0,
                    0,0,0,1);
            cv::Mat rotation_pitch_matrix = (cv::Mat_<double>(4,4) << cos(pitch),0,sin(pitch),0,
                    0,1,0,0,
                    -sin(pitch),0,cos(pitch),0,
                    0,0,0,1);
            cv::Mat rotation_yaw_matrix = (cv::Mat_<double>(4,4) << cos(yaw),-sin(yaw),0,0,
                    sin(yaw),cos(yaw),0,0,
                    0,0,0,0,
                    0,0,0,1);
            // Oritentation matrix is defined as R_yaw * R_pitch * R_roll (XYZ)
            cv::Mat world_Trobot_base = rotation_yaw_matrix * rotation_pitch_matrix * rotation_roll_matrix;
            // Position (x,y,z) of the robot given by the odometry
            world_Trobot_base.at<double>(0,3) = odometry_msg->pose.pose.position.x;
            world_Trobot_base.at<double>(1,3) = odometry_msg->pose.pose.position.y;
            world_Trobot_base.at<double>(2,3) = odometry_msg->pose.pose.position.z;

            // Computing the location of the MCS respect to the world
            cv::Mat world_Tcamera_base = world_Trobot_base * robot_base_Tcamera_base_;

            // Convert ROS image messages to OpenCV matrices
            //cv_bridge::CvImageConstPtr cv_ptr0, cv_ptr1, cv_ptr2, cv_ptr3, cv_ptr4, cv_ptr5;
            std::vector<cv::Mat> images(cam_system_.get_nrCams());
            images[0] = (cv_bridge::toCvShare(msg0, sensor_msgs::image_encodings::BGR8))->image;
            images[1] = (cv_bridge::toCvShare(msg1, sensor_msgs::image_encodings::BGR8))->image;
            images[2] = (cv_bridge::toCvShare(msg2, sensor_msgs::image_encodings::BGR8))->image;
            images[3] = (cv_bridge::toCvShare(msg3, sensor_msgs::image_encodings::BGR8))->image;
            images[4] = (cv_bridge::toCvShare(msg4, sensor_msgs::image_encodings::BGR8))->image;
            images[5] = (cv_bridge::toCvShare(msg5, sensor_msgs::image_encodings::BGR8))->image;
            // Check that all camera images are not empty and saving them into an array
            for(auto &img : images){
                if(img.empty()){
                    b_frame_incomplete = true;
                    break;
                }
            }

            // Saving image data
            if(!b_frame_incomplete){
                MultiFrame frame(images,world_Tcamera_base,0); // TODO: Add timestamp, 0 at the moment
                std::unique_lock<std::mutex> lock(mFrameData);
                current_frame_ = frame;
            }
        }
        /**
         * Callable service to activate the data transfer for the current_frame data to the collision detection
         * thread.
        */
        bool TransferDataCallback(collision_detection_module::TransferData::Request & req,
                                collision_detection_module::TransferData::Response & res){
            // Transferring current image and pose to the collision Detection thread
            std::unique_lock<std::mutex> lock(mFrameData);
            coldetector->transferData(current_frame_, req.send_data);
            res.complete = true;
            return true;
        }

        /**
         *  Loads the camera configuration parameters from the rosparam server.
        */
        void loadParameters(){
            // Loading the extrinsic cayley parameters for each camera.
            int nrCams;
            nh_.getParam("/CameraSystem_nrCams",nrCams);

            // Setting the camera origin location
            double base_x_pos, base_y_pos, base_z_pos;
            nh_.getParam("/cam_origin_pose_x",base_x_pos);
            nh_.getParam("/cam_origin_pose_y",base_y_pos);
            nh_.getParam("/cam_origin_pose_z",base_z_pos);
            // Multi-camera system real position on the robot
            cv::Mat robot_base_Tcamera_base = (cv::Mat_<double>(4,4) << 1,0,0,base_x_pos,
                    0,1,0, base_y_pos,
                    0,0,1,base_z_pos,
                    0,0,0,1);

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
            robot_base_Tcamera_base_ = robot_base_Tcamera_base;
        }
        /**
         * Makes the request to stop all working threads
        */
        void RequestStop(){
            coldetector->StopRequest();
        }

};

int main(int argc, char** argv){
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc,&argv,true);
    google::InstallFailureSignalHandler();

    ros::init(argc,argv,"CollisionNode");
    ros::NodeHandle nh("~");

    CollisionNode my_collision_node(nh);

    ros::spin();
    
    // Stop all thread when ROS stops working
    my_collision_node.RequestStop();

    return 0;
}