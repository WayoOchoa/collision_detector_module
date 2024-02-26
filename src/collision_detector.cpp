#include <iostream>
#include <cmath>
#include "collision_detector.h"

#include "ros/ros.h"

using namespace cv::xfeatures2d;

// PointCloud type definition
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Parameters definition
DECLARE_bool(clahe_processing);
DECLARE_bool(matching_all_vs_all);
DECLARE_bool(point_cloud_in_world);
DECLARE_bool(consider_chassis);
DECLARE_int32(int_epipolar_dst_thr);

namespace coldetector
{
   CollisionDetector::CollisionDetector():
   b_new_data_(false), b_do_clahe_(FLAGS_clahe_processing), b_matching_all_all_(FLAGS_matching_all_vs_all),
   b_pointcloud_in_world_(FLAGS_point_cloud_in_world), b_consider_chassis_b_(FLAGS_consider_chassis)
   {
      // Robot chasis, critical points initialization
      if(b_consider_chassis_b_) assignChassisPoints();
   }

   void CollisionDetector::Run(){
      // Final 3D pts variable
      PointCloud::Ptr reconstructed_point_cloud(new PointCloud);

      while (true) // Runs until program is closed
      {
         if(CheckDataAvailability()){ // Check if there is new data available from the robot
         }
      }
      
   }

   bool CollisionDetector::CheckDataAvailability(){
      std::unique_lock<std::mutex> lock(mReceiveData);
      bool temp = b_new_data_;
      if(b_new_data_){
         current_imgs_frame_ = data_current_frame_imgs;
         current_frame_pose_ = data_current_pose;
         b_new_data_= false;
      }

      return temp;
   }

   void CollisionDetector::assignChassisPoints(){
      // All points assigned to the chassis are w.r.t. the camera system axes. These
      // are defined with the actual values of the AUV.
      cv::Mat point =  (cv::Mat_<double>(3,1) << 0,0,0);
      girona1000_chassis_points_.push_back(point); // Camera system location
      if(true){
          cv::Mat point_a =  (cv::Mat_<double>(3,1) << -0.83,0.5,-1.34);
          cv::Mat point_b =  (cv::Mat_<double>(3,1) << -0.83,-0.5,-1.34);
          cv::Mat point_c =  (cv::Mat_<double>(3,1) << 0.77,-0.5,-1.34);
          cv::Mat point_d =  (cv::Mat_<double>(3,1) << 0.77,0.5,-1.34);
          cv::Mat point_e =  (cv::Mat_<double>(3,1) << -0.83,0,-0.59);
          cv::Mat point_f =  (cv::Mat_<double>(3,1) << 0.72,0,-0.63);
          girona1000_chassis_points_.push_back(point_a); // PointA
          girona1000_chassis_points_.push_back(point_b); // PointB
          girona1000_chassis_points_.push_back(point_c); // PointC
          girona1000_chassis_points_.push_back(point_d); // PointD
          girona1000_chassis_points_.push_back(point_e); // PointE
          girona1000_chassis_points_.push_back(point_f); // PointF
      }else{
          cv::Mat point_a =  (cv::Mat_<double>(3,1) << -1.4,0.5,-0.89);
          cv::Mat point_b =  (cv::Mat_<double>(3,1) << -1.4,-0.5,-0.89);
          cv::Mat point_c =  (cv::Mat_<double>(3,1) << 0.25,-0.5,-0.89);
          cv::Mat point_d =  (cv::Mat_<double>(3,1) << 0.25,0.5,-0.89);
          cv::Mat point_e =  (cv::Mat_<double>(3,1) << -1.4,0,0.15);
          cv::Mat point_f =  (cv::Mat_<double>(3,1) << 0.2,0,0.15);
          girona1000_chassis_points_.push_back(point_a); // PointA
          girona1000_chassis_points_.push_back(point_b); // PointB
          girona1000_chassis_points_.push_back(point_c); // PointC
          girona1000_chassis_points_.push_back(point_d); // PointD
          girona1000_chassis_points_.push_back(point_e); // PointE
          girona1000_chassis_points_.push_back(point_f); // PointF
      }
   }
}