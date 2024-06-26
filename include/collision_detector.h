#pragma once

//System includes
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <numeric>
#include <stdio.h>
#include <mutex>
#include <sys/stat.h>

#include <Eigen/Core>
#include <eigen3/Eigen/Eigenvalues>

//OpenCV includes
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

//PCL library
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point32.h>

// Gflags
#include <gflags/gflags.h>

// Additional libraries
#include "MultiFrame.h"
#include "cSystem.h"

using namespace std;

/**
 * @brief Namespace containing functions to perform the risk assessment process
 * 
 */
namespace coldetector
{
    class cSystem;
    class MultiFrame;

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
            ros::Publisher *pc_test_;

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
            CollisionDetector(cSystem *cam_system,ros::Publisher *pc_pub, ros::Publisher *pc_test);
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
            void transferData(const MultiFrame &F, bool new_data);
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
            void GetRelativeTransformationBetweenFrames(const cv::Mat &base_T_cam, const cv::Mat &world_Tprevious_base,
                                                        const cv::Mat &world_Tcurrent_base, cv::Mat &cam_previous_T_cam_current);
            /**
             * @brief Compute keypoints and its descriptors in two images.
             * @param previous_image_cam_i Image of camera i in the previous frame.
             * @param current_image_cam_i Image of camera i in the currnet frame.
             * @param cam_id Current camera id that is used for feature detection.
             * @param keypoints_previous_i Output vector of cv::Keypoints corresponding to features detected on previous frame.
             * @param keypoints_current_i Output vector of cv::Keypoints corresponding to features detected on current frame.
             * @param descriptors_previous_i Output vector of corresponding descriptors on previous frame.
             * @param descriptors_current_i Output vector of corresponding descriptors on current frame.
            */
            void ComputeFeatures(const cv::Mat &previous_image_cam_i, const cv::Mat &current_image_cam_i, const int &cam_id, std::vector <cv::KeyPoint> &keypoints_previous_i, 
                                std::vector <cv::KeyPoint> &keypoints_current_i, cv::Mat &descriptors_previous_i, cv::Mat &descriptors_current_i);
            /**
             * @brief Compute the Fundamental matrix between two frames given their relative transformation matrix
             * @param current_T_previous relative transformation matrix of previous frame w.r.t the current frame.
             * @param cam_K The instrinsic parameters of the camera.
             * @param current_F_previous Output Fundamental matrix of the previous frame w.r.t. the current frame.
            */
            void ComputeFundamentalMatrix(const cv::Mat & current_T_previous, const cv::Mat cam_K, cv::Mat &current_F_previous);
            /**
             * @brief Performs Brute force matching (all vs all) from a set of previous_keypoints and current_keypoints
             * @param descriptors_previous_i Descriptors computed on the previous frame.
             * @param descriptors_current_i Descriptors computed on the current frame.
             * @param best_matches Output vector with matches that satisfy the Lowe's ratio test.
            */
            void ComputeMatchesAllvsAll(const cv::Mat &descriptors_previous_i, const cv::Mat &descriptors_current_i, std::vector<cv::DMatch>& best_matches);
            /**
             * @brief Uses epipolar geometry to filter the matches from a set of two images.
             * @param keypoints_previous_i Features detected on previous frame.
             * @param keypoints_current_i Features detected on current frame.
             * @param best_matches Set of feature matches between images.
             * @param F_matrix Fundamental matrix of the previous frame w.r.t. the current frame.
             * @param img_size Size of the image.
             * @param filtered_matches Output vector with all the feature matches that satisfy the epipolar constraint.
            */
            void FilterMatchesByEpipolarConstrain(const std::vector <cv::KeyPoint> &keypoints_previous_i, const std::vector <cv::KeyPoint> &keypoints_current_i, 
                                                const std::vector<cv::DMatch> &best_matches, const cv::Mat &F_matrix, const cv::Size2i &img_size, std::vector<cv::DMatch>& filtered_matches);
            /**
             * @brief Computes the cosine similarity between the rays of point correpondences. If the rays are similar (parallel) in direction they are filtered and not count for triangulation.
             * @param keypoints_frame1 Features detected on frame 1.
             * @param keypoints_frame2 Features detected on frame 2.
             * @param world_Tprevious_base Transformation of the camera origin w.r.t the world on the previous frame.
             * @param world_Tcurrent_base Transformation of the camera origin w.r.t the world on the current frame.
             * @param matches Vector of the computed matches between frames.
             * @param filtered_matches Output vector with all the feature matches that satisfy the cosine similarity constraint
             */
            void filterMatchesByAngleComparison(const std::vector<cv::KeyPoint> &keypoints_frame1, const std::vector<cv::KeyPoint> &keypoints_frame2, const cv::Mat &world_Tprevious_base,
                                                const cv::Mat &world_Tcurrent_base, const cv::Mat cam_K, const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch>& filtered_matches);
            /**
             * @brief Transforms a vector of DMatch type into a Point2d object
             * @param keypoints_frame1 Features detected on frame 1.
             * @param keypoints_frame2 Features detected on frame 2.
             * @param matches Vector of the computed matches between frames.
             * @param points_frame1 Output vector of cv::Point2f 2D points on frame 1.
             * @param points_frame2 Output vector of cv::Point2f 2D points on frame 2.
            */
            void FromMatchesToVectorofPoints(const std::vector<cv::KeyPoint> &keypoints_frame1, const std::vector<cv::KeyPoint> &keypoints_frame2,
                                    const std::vector<cv::DMatch> &matches, std::vector <cv::Point2f> &points_frame1,std::vector <cv::Point2f> &points_frame2);
            /**
             * @brief Computes the orthogonal distances from a 2D point to a line
             * @param point The 2D point in the image.
             * @param epiline The corresponding epipolar line.
             * @return distance The orthogonal distance from the point to the line.
            */
            double DistancePointToLine(const cv::Point2f point, const cv::Vec3f epiline);
            /**
             * @brief Triangulate 3D points from a set of corresponding 2D matches.
             * @param[in] base_Tcam Extrinsic camera parametes.
             * @param[in] cam_K The instrinsic parameters of the camera.
             * @param[in] current_T_previous The relative transformation of image 2 w.r.t image 1.
             * @param[in] keypoints_previous_i Features detected on image 1.
             * @param[in] keypoints_current_i Features detected on image 2.
             * @param[in] descriptors_previous_i Descriptors computed on the previous frame.
             * @param[in] descriptors_current_i Descriptors computed on the current frame.
             * @param[in] filtered_matches Feature matches between image 1 and image 2.
             * @param[in] world_Tcurrent_base Transformation of the camera i w.r.t the world.
             * @param[in] b_to_world Flag to indicate if the final point cloud should be referenced to the world.
             * @param[out] triangulated_3dpoints Output array with the correspondent 3D point
             * @param[out] final_descriptors Output matrix of the correspondent descriptors from the 3D points that were computed.
            */
            void TriangulatePoints(const cv::Mat &base_Tcam, const cv::Mat cam_K, const cv::Mat & current_T_previous, const std::vector <cv::KeyPoint> &keypoints_previous_i, 
                                const std::vector <cv::KeyPoint> &keypoints_current_i, const cv::Mat &descriptors_previous_i, const cv::Mat &descriptors_current_i, 
                                const std::vector<cv::DMatch>& filtered_matches, const cv::Mat &world_Tcurrent_base, 
                                bool b_to_world, std::vector<cv::Mat> &final_3d_pts, std::vector<double> &points_keypoints, cv::Mat &final_descriptors);
            /**
             * @brief Computes the Projection matrices of two frames.
             * @param cam_K The instrinsic parameters of the camera.
             * @param current_T_previous The relative transformation of image 2 w.r.t image 1.
             * @param P_previous Output Projection matrix of the previous frame.
             * @param P_current Output Projection matrix of the current frame.
            */
            void ComputeProjectionMatrices(const cv::Mat cam_K, const cv::Mat &current_T_previous, cv::Mat &P_previous, cv::Mat &P_current);
            /**
             * @brief Adjust a set of cv:Point2f vector arrays into an InputArrayOfArrays object.
             * @param points_frame1 of cv::Point2f 2D points on frame 1.
             * @param points_frame2 of cv::Point2f 2D points on frame 2.
             * @param array_of_points Set of points converted to an std::vector of cv::Mat.
            */
            void GetArrayOfPoints(std::vector<cv::Point2f> &points_frame1, std::vector<cv::Point2f> &points_frame2, std::vector<cv::Mat> &array_of_points);
            /**
             * @brief Performs the triangulation of 3D points given a set of 2D correspondences between two images and their respective Projection matrices
             * @param _points2d 2D point correspondences from image left and right
             * @param _projection_matrices Projection matrices of the two views
             * @param pixel_range How many pixels are considered from the original 2d point to extract the uncertainty.
             * @param _points3d Output matrix with the corresponding triangulated 3D points
            */
            void triangulatePoints2Views(cv::InputArrayOfArrays _points2d, cv::InputArrayOfArrays _projection_matrices, const int &pixel_range, cv::OutputArray _points3d);
            /**
             * @brief Triangulate points using DLT algorihtm
             * @param xl Input vector with first 2d point.
             * @param xr Input vector with second 2d point.
             * @param Pl Input 3x4 first projection matrix.
             * @param Pr Input 3x4 second projection matrix.
             * @param point3d Output vector with computed 3d point. 
             * 
            */
            void triangulateDLT(const cv::Vec2d &xl, const cv::Vec2d &xr, const cv::Matx34d &Pl, const cv::Matx34d &Pr, cv::Vec3d &point3d);
            /**
             * @brief Converts point coordinates from homogeneours to euclidean coordinates
             * @param src Input vector of N-dimensional points.
             * @param dst Output vector of N-1-dimensional points.
             */
            void homogeneousToEuclidean(cv::InputArray src, cv::OutputArray dst);
            /**
             * @brief From OpenCv sfm library: Converts points from Euclidean to homogeneous space. E.g., ((x,y)->(x,y,1))
             * @param src Input vector of N-dimensional points.
             * @param dst Output vector of N+1-dimensional points.
             */
            void euclideanToHomogeneous(cv::InputArray src, cv::OutputArray dst);
            /**
             * @brief From OpenCv sfm library: Get projection matrix P from K, R and t.
             * @param K Input 3x3 camera matrix \f$K = \vecthreethree{f_x}{0}{c_x}{0}{f_y}{c_y}{0}{0}{1}\f$.
             * @param R Input 3x3 rotation matrix.
             * @param t Input 3x1 translation vector.
             * @param P Output 3x4 projection matrix.
             * 
             * This function estimate the projection matrix by solving the following equation: \f$P = K * [R|t]\f$
             */
            void projectionFromKRt(cv::InputArray K, cv::InputArray R, cv::InputArray t, cv::OutputArray P);

            // Data memebers
            bool b_new_data_;
            // Variables used for sharing and get data from the system
            MultiFrame data_current_frame_imgs;
            cv::Matx<double, 4, 4> data_current_pose;

            /**
             * @brief Stops the thread when requested from the main program
            */
            void StopRequest();

            // Mutex variables
            std::mutex mReceiveData;
            std::mutex mMutexStop;

    };
}