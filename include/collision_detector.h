#pragma once

//System includes
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <stdio.h>
#include <mutex>
#include <sys/stat.h>

//OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>

//PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>

// Gflags
#include <gflags/gflags.h>

// Additional libraries
#include "MultiFrame.h"
#include "cSystem.h"

using namespace std;

namespace coldetector
{
    class CollisionDetector{
        private:
            vector<cv::Mat> girona1000_chassis_points_; 

            /**
             * Variable that stores the new acquired/tracked frame
            */
            MultiFrame new_frame_data_;
            cv::Matx<double, 4, 4> new_frame_pose_;
            /**
             * Stores the current MultiFrame data received from the system at time t
            */
            MultiFrame current_imgs_frame_;
            /**
             * Stores the previous MultiFrame data received from the system at (t-1)
            */
            MultiFrame previous_imgs_frame_;
            /**
             * Stores the frame current pose
            */
            cv::Matx<double, 4, 4> current_frame_pose_;
            /**
             * Stores the previous frame pose
            */
            cv::Matx<double, 4, 4> previous_frame_pose_;
            // Camera system configuration parameters
            cSystem* cam_system_;
        
        public:
            // Constructor
            CollisionDetector(cSystem *cam_system);
            ~CollisionDetector(){};

            //Parameters
            bool b_do_clahe_; //Images Pre-processing
            bool b_matching_all_all_; // feature matching: all against all
            bool b_pointcloud_in_world_; // set 3D points in world coordinate frame
            bool b_consider_chassis_b_; // Consider critical points for risk assessment

            /**
             * @brief Function that initialize a set of 3D points for the AUV
             * chassis. These points are used later for assesing the risk.
            */
            void assignChassisPoints();
            /**
             * @brief Obtains a pair of images and the estimated poses (vSLAm, INS data)
             * and processed them to obtain the 3D points of the robot surroundings.
            */
            void Run();
            /**
             * @brief Checks if new image and pose data is available from the system
             * and copies them for processing.
             * @return temp Flag that indicates wether if new data has been receieved or not.
            */
            bool CheckDataAvailability();
            /**
             * @brief Transfers data from the main program to the collision detection thread.
             * @param F MultiFrame object that contains an array of N images, corresponding to each
             * camera.
             * @param new_data Flag that indicates that new data has been received.
            */
            void transferData(MultiFrame &F, bool new_data);

            // Data memebers
            bool b_new_data_;
            // Variables used for sharing and get data from the system
            MultiFrame data_current_frame_imgs;
            cv::Matx<double, 4, 4> data_current_pose;

            // Mutex variables
            std::mutex mReceiveData;

    };
}