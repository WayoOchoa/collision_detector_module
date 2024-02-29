#include <iostream>
#include <cmath>
#include "collision_detector.h"

#include "ros/ros.h"

using namespace cv;
using namespace cv::xfeatures2d;

// PointCloud type definition
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Parameters definition
DECLARE_bool(clahe_processing);
DECLARE_bool(matching_all_vs_all);
DECLARE_bool(point_cloud_in_world);
DECLARE_bool(consider_chassis);
DECLARE_int32(int_epipolar_dst_thr);
DECLARE_string(detector_type);
DECLARE_int32(number_of_features);
DECLARE_int32(min_Hessian);
DECLARE_int32(nOctaves);
DECLARE_int32(nOctaveLayers);
DECLARE_double(contrastThreshold);
DECLARE_double(edgeThreshold);
DECLARE_double(sigma);


namespace coldetector
{
   CollisionDetector::CollisionDetector(cSystem *cam_system):
   b_new_data_(false), b_do_clahe_(FLAGS_clahe_processing), b_matching_all_all_(FLAGS_matching_all_vs_all), bFinishRequested_(false),
   b_pointcloud_in_world_(FLAGS_point_cloud_in_world), b_consider_chassis_b_(FLAGS_consider_chassis), cam_system_(cam_system)
   {
      // Robot chasis, critical points initialization
      if(b_consider_chassis_b_) assignChassisPoints();
   }

   void CollisionDetector::Run(){
      // Final 3D pts variable
      PointCloud::Ptr reconstructed_point_cloud(new PointCloud);
      int number_cams = cam_system_->get_nrCams();

      while (true) // Runs until program is closed
      {
         bool b_new_pc_data = false; // Variable to control if a PointCloud was successfully computed
         if(CheckDataAvailability()){ // Check if there is new data available from the robot
            current_imgs_frame_ = new_frame_data_; // Set the current frame to the newly received data.
            reconstructed_point_cloud.reset(new PointCloud);

            // Checking that we have already received two frames from the system
            if(!previous_imgs_frame_.b_empty){
               std::vector<cv::Mat> final_3d_points; // Stores the triangulated points

               /// Get camera base poses w.r.t. the world
               cv::Mat world_Tcurrent_base = cv::Mat(current_imgs_frame_.world_Tcamera_base_);
               cv::Mat world_Tprevious_base = cv::Mat(previous_imgs_frame_.world_Tcamera_base_);

               /// Process camera data
               for(int cam_id = 0; cam_id < number_cams; cam_id++){
                  // 1. Get the transformation matrix between the previous and current frame
                  // Extrinsic calibration matrix of cam_id
                  cv::Mat base_Tcam = cv::Mat(cam_system_->getM_c(cam_id));
                  // Compute the relative transformation between frames
                  cv::Mat cam_previous_Tcam_current(4,4,CV_64F);
                  GetRelativeTransformationBetweenFrames(base_Tcam,world_Tprevious_base,world_Tcurrent_base,
                                                         cam_previous_Tcam_current);
                  cv::Mat cam_current_Tcam_previous = cam_previous_Tcam_current.inv(); // TODO: Check that it is a copy and not a reference

                  // 2. Image pre-processing: The image is processed to enhance the contrast and revognize
                  // more features in the image
                  cv::Mat previous_image_cam_i, current_image_cam_i;
                  if(FLAGS_clahe_processing){
                     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                     cout <<"C1\n";
                     clahe->setClipLimit(2.0);
                     cout <<"C2\n";
                     clahe->apply(previous_imgs_frame_.images_[cam_id], previous_image_cam_i);
                     clahe->apply(current_imgs_frame_.images_[cam_id], current_image_cam_i);
                  }else{
                     previous_image_cam_i = previous_imgs_frame_.images_[cam_id];
                     current_image_cam_i = current_imgs_frame_.images_[cam_id];
                  }

                  // 3. Feature detection
                  // Detect keypoints in both images and compute its descriptors
                  std::vector<cv::KeyPoint> keypoints_previous_i, keypoints_current_i;
                  cv::Mat descriptors_previous_i, descriptors_current_i;
                  ComputeFeatures(keypoints_previous_i,keypoints_current_i,descriptors_previous_i,descriptors_current_i);

                  continue;
               }
            }
            FramesUpdate(current_imgs_frame_);  
         }

         // Stops this thread if requested by the main program
         if(CheckifStop()){
            break;
         }
      }
      
   }

   bool CollisionDetector::CheckDataAvailability(){
      std::unique_lock<std::mutex> lock(mReceiveData);
      bool temp = b_new_data_;
      if(b_new_data_){
         new_frame_data_ = data_current_frame_imgs;
         b_new_data_= false;
      }

      return temp;
   }

   void CollisionDetector::transferData(MultiFrame &F, bool new_data){
      std::unique_lock<std::mutex> lock(mReceiveData);
      b_new_data_ = new_data;
      data_current_frame_imgs = F;
   }

   void CollisionDetector::GetRelativeTransformationBetweenFrames(cv::Mat &base_T_cam, cv::Mat &world_Tprevious_base,
                                                              cv::Mat &world_Tcurrent_base, cv::Mat &cam_previous_T_cam_current){

      cv::Mat world_Tprevious_cam = world_Tprevious_base * base_T_cam;
      cv::Mat world_Tcurrent_cam = world_Tcurrent_base * base_T_cam;

      // Getting the transformation
      cam_previous_T_cam_current = world_Tprevious_cam.inv() * world_Tcurrent_cam;
   }

   void CollisionDetector::ComputeFeatures(std::vector <cv::KeyPoint> &keypoints_previous_i, std::vector <cv::KeyPoint> &keypoints_current_i,
                                          cv::Mat &descriptors_previous_i, cv::Mat &descriptors_current_i){
      if(FLAGS_detector_type == "SIFT"){
         //cv::Ptr<SIFT> detector = SIFT::create(FLAGS_number_of_features,FLAGS_nOctaveLayers,FLAGS_contrastThreshold,
         //                                             FLAGS_edgeThreshold, FLAGS_sigma);
      }else if(FLAGS_detector_type == "SURF"){
         //cv::Ptr<SURF> detector = SURF::create(FLAGS_min_Hessian,FLAGS_nOctaves,FLAGS_nOctaveLayers,false);
      }
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

   // Update the Keyframes of a moment in time
   void CollisionDetector::FramesUpdate(MultiFrame &current_frame)
   {
      std::unique_lock<std::mutex> lock(mReceiveData);
      previous_imgs_frame_ = current_frame;
      previous_imgs_frame_.b_empty = false;
	}

   void CollisionDetector::StopRequest(){
        unique_lock<mutex> lock(mMutexStop);
        bFinishRequested_ = true;
    }

    bool CollisionDetector::CheckifStop(){
        unique_lock<mutex> lock(mMutexStop);
        return bFinishRequested_;
    }
}