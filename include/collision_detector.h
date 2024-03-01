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
#include <opencv2/opencv.hpp>
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
            /**
             * Stores the current MultiFrame data received from the system at time t
            */
            MultiFrame current_imgs_frame_;
            /**
             * Stores the previous MultiFrame data received from the system at (t-1)
            */
            MultiFrame previous_imgs_frame_;
            // Camera system configuration parameters
            cSystem* cam_system_;
            // Main thread publishers
            ros::Publisher *pc_pub_;

            /**
             * Flag that indicates if a stop has been requested
            */
            bool bFinishRequested_;

            /**
             * Checks if a request to stop the program has been made
            */
            bool CheckifStop();
        
        public:
            // Constructor
            CollisionDetector(cSystem *cam_system,ros::Publisher *pc_pub);
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
            /**
             * @brief Updates the previous_frame data with the current_frame that was already processed.
             * @param current_frame Frame data corresponding to the newly processed data.
            */
            void FramesUpdate(MultiFrame &current_frame);
            /**
             * @brief Computes the relative transformation between two frames.
             * @param base_T_cam Transformation of the camera i w.r.t to the camera origin.
             * @param world_Tprevious_base Transformation of the camera origin w.r.t the world on the previous frame.
             * @param world_Tcurrent_base Transformation of the camera origin w.r.t the world on the current frame.
             * @param cam_previous_T_cam_current Output relative transformation matrix.
            */
            void GetRelativeTransformationBetweenFrames(cv::Mat &base_T_cam, cv::Mat &world_Tprevious_base,
                                                        cv::Mat &world_Tcurrent_base, cv::Mat &cam_previous_T_cam_current);
            /**
             * @brief Compute keypoints and its descriptors in two images.
             * @param previous_image_cam_i Image of camera i in the previous frame.
             * @param current_image_cam_i Image of camera i in the currnet frame.
             * @param keypoints_previous_i Output vector of cv::Keypoints corresponding to features detected on previous frame.
             * @param keypoints_current_i Output vector of cv::Keypoints corresponding to features detected on current frame.
             * @param descriptors_previous_i Output vector of corresponding descriptors on previous frame.
             * @param descriptors_current_i Output vector of corresponding descriptors on current frame.
            */
            void ComputeFeatures(cv::Mat &previous_image_cam_i, cv::Mat &current_image_cam_i, std::vector <cv::KeyPoint> &keypoints_previous_i, 
                                std::vector <cv::KeyPoint> &keypoints_current_i, cv::Mat &descriptors_previous_i, cv::Mat &descriptors_current_i);
            /**
             * @brief Compute the Fundamental matrix between two frames given their relative transformation matrix
             * @param current_T_previous relative transformation matrix of previous frame w.r.t the current frame.
             * @param cam_K The instrinsic parameters of the camera.
             * @param current_F_previous Output Fundamental matrix of the previous frame w.r.t. the current frame.
            */
            void ComputeFundamentalMatrix(cv::Mat & current_T_previous, cv::Mat cam_K, cv::Mat &current_F_previous);
            /**
             * @brief Performs Brute force matching (all vs all) from a set of previous_keypoints and current_keypoints
             * @param descriptors_previous_i Descriptors computed on the previous frame.
             * @param descriptors_current_i Descriptors computed on the current frame.
             * @param best_matches Output vector with matches that satisfy the Lowe's ratio test.
            */
            void ComputeMatchesAllvsAll(cv::Mat &descriptors_previous_i, cv::Mat &descriptors_current_i, std::vector<cv::DMatch>& best_matches);
            /**
             * @brief Uses epipolar geometry to filter the matches from a set of two images.
             * @param keypoints_previous_i Features detected on previous frame.
             * @param keypoints_current_i Features detected on current frame.
             * @param best_matches Set of feature matches between images.
             * @param F_matrix Fundamental matrix of the previous frame w.r.t. the current frame.
             * @param filtered_matches Output vector with all the feature matches that satisfy the epipolar constraint.
            */
            void FilterMatchesByEpipolarConstrain(std::vector <cv::KeyPoint> &keypoints_previous_i, std::vector <cv::KeyPoint> &keypoints_current_i, 
                                                std::vector<cv::DMatch> &best_matches, cv::Mat &F_matrix, std::vector<cv::DMatch>& filtered_matches);
            /**
             * @brief Transforms a vector of DMatch type into a Point2d object
             * @param keypoints_frame1 Features detected on frame 1.
             * @param keypoints_frame2 Features detected on frame 2.
             * @param matches Vector of the computed matches between frames.
             * @param points_frame1 Output vector of cv::Point2f 2D points on frame 1.
             * @param points_frame2 Output vector of cv::Point2f 2D points on frame 2.
            */
            void FromMatchesToVectorofPoints(std::vector<cv::KeyPoint> &keypoints_frame1, std::vector<cv::KeyPoint> &keypoints_frame2,
                                    std::vector<cv::DMatch> &matches, std::vector <cv::Point2f> &points_frame1,std::vector <cv::Point2f> &points_frame2);
            /**
             * @brief Computes the orthogonal distances from a 2D point to a line
             * @param point The 2D point in the image.
             * @param epiline The corresponding epipolar line.
             * @return distance The orthogonal distance from the point to the line.
            */
            double DistancePointToLine(cv::Point2f point, cv::Vec3f epiline);
            /**
             * @brief Triangulate 3D points from a set of corresponding 2D matches.
             * @param base_Tcam Extrinsic camera parametes.
             * @param cam_K The instrinsic parameters of the camera.
             * @param current_T_previous The relative transformation of image 2 w.r.t image 1.
             * @param keypoints_previous_i Features detected on image 1.
             * @param keypoints_current_i Features detected on image 2.
             * @param filtered_matches Feature matches between image 1 and image 2.
             * @param world_Tcurrent_base Transformation of the camera i w.r.t the world.
             * @param b_to_world Flag to indicate if the final point cloud should be referenced to the world.
             * @param triangulated_3dpoints Output array with the correspondent 3D point
            */
            void TriangulatePoints(cv::Mat &base_Tcam, cv::Mat cam_K, cv::Mat & current_T_previous, std::vector <cv::KeyPoint> &keypoints_previous_i, 
                                std::vector <cv::KeyPoint> &keypoints_current_i, std::vector<cv::DMatch>& filtered_matches, cv::Mat &world_Tcurrent_base, 
                                bool b_to_world, std::vector<cv::Mat> &final_3d_pts);
            /**
             * @brief Computes the Projection matrices of two frames.
             * @param cam_K The instrinsic parameters of the camera.
             * @param current_T_previous The relative transformation of image 2 w.r.t image 1.
             * @param P_previous Output Projection matrix of the previous frame.
             * @param P_current Output Projection matrix of the current frame.
            */
            void ComputeProjectionMatrices(cv::Mat cam_K, cv::Mat &current_T_previous, cv::Mat &P_previous, cv::Mat &P_current);
            /**
             * @brief Adjust a set of cv:Point2f vector arrays into an InputArrayOfArrays object.
             * @param points_frame1 of cv::Point2f 2D points on frame 1.
             * @param points_frame2 of cv::Point2f 2D points on frame 2.
             * @param array_of_points Set of points converted to an std::vector of cv::Mat.
            */
            void GetArrayOfPoints(std::vector<cv::Point2f> &points_frame1, std::vector<cv::Point2f> &points_frame2, std::vector<cv::Mat> &array_of_points);

            // Data memebers
            bool b_new_data_;
            // Variables used for sharing and get data from the system
            MultiFrame data_current_frame_imgs;
            cv::Matx<double, 4, 4> data_current_pose;

            /**
             * Stops the thread when requested from the main program
            */
            void StopRequest();

            // Mutex variables
            std::mutex mReceiveData;
            std::mutex mMutexStop;

    };
}