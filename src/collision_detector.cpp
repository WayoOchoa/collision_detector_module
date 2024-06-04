#include <iostream>
#include <cmath>
#include "collision_detector.h"

#include "ros/ros.h"
#include "collision_detection_module/DescribedPointCloud.h"

using namespace cv;
using namespace cv::xfeatures2d;

// PointCloud type definition
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

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
   // Overloaded function of homogeneous to Euclidean
   template<typename T>
   void homogeneousToEuclidean(const cv::Mat & X_, cv::Mat & x_){
     int d = X_.rows - 1;

     const cv::Mat_<T> & X_rows = X_.rowRange(0,d);
     const cv::Mat_<T> h = X_.row(d);

     const T * h_ptr = h[0], *h_ptr_end = h_ptr + h.cols;
     const T * X_ptr = X_rows[0];
     T * x_ptr = x_.ptr<T>(0);
     for (; h_ptr != h_ptr_end; ++h_ptr, ++X_ptr, ++x_ptr)
     {
       const T * X_col_ptr = X_ptr;
       T * x_col_ptr = x_ptr, *x_col_ptr_end = x_col_ptr + d * x_.step1();
       for (; x_col_ptr != x_col_ptr_end; X_col_ptr+=X_rows.step1(), x_col_ptr+=x_.step1() )
         *x_col_ptr = (*X_col_ptr) / (*h_ptr);
     }
   }

   CollisionDetector::CollisionDetector(cSystem *cam_system, ros::Publisher* pc_pub, ros::Publisher *pc_test):
   b_new_data_(false), b_do_clahe_(FLAGS_clahe_processing), b_matching_all_all_(FLAGS_matching_all_vs_all), bFinishRequested_(false),
   b_pointcloud_in_world_(FLAGS_point_cloud_in_world), b_consider_chassis_b_(FLAGS_consider_chassis), cam_system_(cam_system), pc_pub_(pc_pub),
   pc_test_(pc_test)
   {
      // Robot chasis, critical points initialization
      if(b_consider_chassis_b_) assignChassisPoints();
      previous_imgs_frame_.b_empty = true;
      current_imgs_frame_.b_empty = true;
   }

   void CollisionDetector::Run(){
      // Final 3D pts variable
      PointCloud::Ptr reconstructed_point_cloud(new PointCloud);
      int number_cams = cam_system_->get_nrCams();

      while(true) // Runs until program is closed
      {
         bool b_new_pc_data = false; // Variable to control if a PointCloud was successfully computed
         if(CheckDataAvailability()){ // Check if there is new data available from the robot
            {
               std::lock_guard<std::mutex> lock(mReceiveData);
               current_imgs_frame_ = new_frame_data_; // Set the current frame to the newly received data.
            }
            reconstructed_point_cloud.reset(new PointCloud);

            // Checking that we have already received two frames from the system
            if(!previous_imgs_frame_.b_empty){
               std::vector<cv::Mat> final_3d_points; // Stores the triangulated points
               cv::Mat points_descriptors; // The descriptor corresponding to each triangulated 3D point
               std::vector<double> points_keypoints; // The 2D locations of each corresponding 3D point
               std::vector<int> cam_num_features; // How many features where considered for camera i

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
                  GetRelativeTransformationBetweenFrames(base_Tcam, world_Tprevious_base, world_Tcurrent_base,
                                                         cam_previous_Tcam_current);
                  cv::Mat cam_current_Tcam_previous = cam_previous_Tcam_current.inv();

                  // 2. Image pre-processing: The image is processed to enhance the contrast and revognize
                  // more features in the image
                  cv::Mat previous_image_cam_i, current_image_cam_i;
                  cv::Mat gray_previous_image_cam_i, gray_current_image_cam_i;
                  cv::cvtColor(previous_imgs_frame_.images_[cam_id],gray_previous_image_cam_i,COLOR_BGR2GRAY);
                  cv::cvtColor(current_imgs_frame_.images_[cam_id],gray_current_image_cam_i,COLOR_BGR2GRAY);
                  if(FLAGS_clahe_processing){
                     cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
                     clahe->setClipLimit(2.0);
                     clahe->apply(gray_previous_image_cam_i, previous_image_cam_i);
                     clahe->apply(gray_current_image_cam_i, current_image_cam_i);
                  }else{
                     previous_image_cam_i = gray_previous_image_cam_i;
                     current_image_cam_i = gray_current_image_cam_i;
                  }

                  // 3. Feature detection and matching
                  // Detect keypoints in both images and compute its descriptors
                  std::vector<cv::KeyPoint> keypoints_previous_i, keypoints_current_i;
                  cv::Mat descriptors_previous_i, descriptors_current_i;
                  ComputeFeatures(previous_image_cam_i, current_image_cam_i, cam_id, keypoints_previous_i,keypoints_current_i,descriptors_previous_i,
                                 descriptors_current_i);
                  // Match keypoints between the two frames
                  std::vector<cv::DMatch> best_matches;
                  std::vector<cv::DMatch> pre_filtered_matches;
                  std::vector<cv::DMatch> filtered_matches;
                  // Compute the fundamental matrix between the frames
                  cv::Mat current_Fprevious(3,3,CV_64F);
                  ComputeFundamentalMatrix(cam_current_Tcam_previous, cam_system_->getK_c(cam_id), 
                                       current_Fprevious); // where cam_system_->getK_c(cam_id) is the intrinsic matrix of cam_id

                  if(FLAGS_matching_all_vs_all){
                     ComputeMatchesAllvsAll(descriptors_previous_i,descriptors_current_i,best_matches);
                     // Filtering points which rays are almost parallel (which will result in bad triangulated points)
                     filterMatchesByAngleComparison(keypoints_previous_i, keypoints_current_i, world_Tprevious_base, world_Tcurrent_base, cam_system_->getK_c(cam_id), best_matches, pre_filtered_matches);
                     if(pre_filtered_matches.size() > 0) {
                        static cv::Size2i img_size = previous_image_cam_i.size();
                        // Filtering the matches using eipolar geometry constraints
                        FilterMatchesByEpipolarConstrain(keypoints_previous_i,keypoints_current_i,pre_filtered_matches,current_Fprevious, img_size,filtered_matches);
                     }                
                  }else{
                     // TODO: Epipolar matching
                  }
                  

                  // 4. Triangulation step
                  if(filtered_matches.size() > 0){
                     TriangulatePoints(base_Tcam, cam_system_->getK_c(cam_id), cam_current_Tcam_previous, keypoints_previous_i, 
                                       keypoints_current_i, descriptors_previous_i, descriptors_current_i,
                                       filtered_matches, world_Tcurrent_base, true, final_3d_points, points_keypoints, points_descriptors);
                     cam_num_features.push_back(final_3d_points.size());
                  }
               }

               // Convert Mat of Points into PointCloud variable
               if(final_3d_points.size() > 200){ // Not enough points were triangulated
                  // Prepare data for publishing it into ROS
                  // Point Cloud conversion
                  collision_detection_module::DescribedPointCloud msg;
                  sensor_msgs::PointCloud2 pcl2_msg;
                  sensor_msgs::PointCloud pcl_msg;

                  msg.header.stamp = ros::Time::now();
                  
                  for(int p = 0; p < final_3d_points.size(); p++){
                     geometry_msgs::Point32 point;
                     point.x = final_3d_points[p].at<double>(0,0);
                     point.y = final_3d_points[p].at<double>(1,0);
                     point.z = final_3d_points[p].at<double>(2,0);
                     pcl_msg.points.push_back(point);
                  }
                  pcl_msg.header.frame_id = "world_ned";
                  sensor_msgs::convertPointCloudToPointCloud2(pcl_msg,pcl2_msg);
                  msg.points3d = pcl2_msg;

                  // Converting descriptors
                  msg.num_descriptors = points_descriptors.rows;
                  msg.descriptor_length = points_descriptors.cols;
                  msg.cam_num_features = cam_num_features;
                  
                  msg.descriptors.assign(points_descriptors.begin<float>(),points_descriptors.end<float>());

                  // Passing keypoints extracted
                  msg.keypoints = points_keypoints;

                  // Converting current pose
                  for(int i = 0; i < world_Tcurrent_base.total(); i++){
                     msg.cam_world_pose[i] = *(world_Tcurrent_base.begin<double>()+i);
                  }

                  // Publishing data
                  pc_pub_->publish(msg);
                  pc_test_->publish(pcl2_msg);
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
         std::lock_guard<std::mutex> lock(mReceiveData);
         bool temp = b_new_data_;
         if(b_new_data_){
            new_frame_data_ = data_current_frame_imgs;
            b_new_data_= false;
         }
      return temp;
   }

   void CollisionDetector::transferData(const MultiFrame &F, bool new_data){
      {  
         std::lock_guard<std::mutex> lock(mReceiveData);
         b_new_data_ = new_data;
         data_current_frame_imgs = F;
      }
   }

   void CollisionDetector::GetRelativeTransformationBetweenFrames(const cv::Mat &base_T_cam, const cv::Mat &world_Tprevious_base,
                                                              const cv::Mat &world_Tcurrent_base, cv::Mat &cam_previous_T_cam_current){
      cv::Mat world_Tprevious_cam = world_Tprevious_base * base_T_cam;
      cv::Mat world_Tcurrent_cam = world_Tcurrent_base * base_T_cam;

      // Getting the transformation
      cam_previous_T_cam_current = world_Tprevious_cam.inv() * world_Tcurrent_cam;
   }

   void CollisionDetector::ComputeFeatures(const cv::Mat &previous_image_cam_i, const cv::Mat &current_image_cam_i,const int &cam_id, std::vector <cv::KeyPoint> &keypoints_previous_i, 
                           std::vector <cv::KeyPoint> &keypoints_current_i, cv::Mat &descriptors_previous_i, cv::Mat &descriptors_current_i){
      // Obtains the keypoints and descriptors of the previous_image_frame and current_image_frame
      if(FLAGS_detector_type == "SIFT"){
         cv::Ptr<SIFT> detector = SIFT::create(FLAGS_number_of_features,FLAGS_nOctaveLayers,FLAGS_contrastThreshold,
                                                      FLAGS_edgeThreshold, FLAGS_sigma);
         // Compute keypoints and descriptors only if they haven't been computed in previous steps                                                    
         if(previous_imgs_frame_.frame_keypoints_[cam_id].empty() || previous_imgs_frame_.frame_keypoints_[cam_id].size()<6){
            detector->detectAndCompute(previous_image_cam_i,cv::Mat(), keypoints_previous_i, descriptors_previous_i);
            previous_imgs_frame_.frame_keypoints_[cam_id] = keypoints_previous_i;
            previous_imgs_frame_.frame_descriptors_[cam_id] = descriptors_previous_i;            
         }else{
            keypoints_previous_i = previous_imgs_frame_.frame_keypoints_[cam_id];
            descriptors_previous_i = previous_imgs_frame_.frame_descriptors_[cam_id];
         }
         // Compute new frame keypoints
         detector->detectAndCompute(current_image_cam_i,cv::Mat(), keypoints_current_i, descriptors_current_i);
         current_imgs_frame_.frame_keypoints_[cam_id] = keypoints_current_i;
         current_imgs_frame_.frame_descriptors_[cam_id] = descriptors_current_i;

      }else if(FLAGS_detector_type == "SURF"){
         cv::Ptr<SURF> detector = SURF::create(FLAGS_min_Hessian,FLAGS_nOctaves,FLAGS_nOctaveLayers,false);
         // Compute keypoints and descriptors only if they haven't been computed in previous steps
         if(previous_imgs_frame_.frame_keypoints_[cam_id].empty() || previous_imgs_frame_.frame_keypoints_[cam_id].size()<6){
            detector->detectAndCompute(previous_image_cam_i,cv::Mat(), keypoints_previous_i, descriptors_previous_i);
            previous_imgs_frame_.frame_keypoints_[cam_id] = keypoints_previous_i;
            previous_imgs_frame_.frame_descriptors_[cam_id] = descriptors_previous_i;
         }else{
            keypoints_previous_i = previous_imgs_frame_.frame_keypoints_[cam_id];
            descriptors_previous_i = previous_imgs_frame_.frame_descriptors_[cam_id];
         }
         // Compute new frame keypoints
         detector->detectAndCompute(current_image_cam_i,cv::Mat(), keypoints_current_i, descriptors_current_i);
         current_imgs_frame_.frame_keypoints_[cam_id] = keypoints_current_i;
         current_imgs_frame_.frame_descriptors_[cam_id] = descriptors_current_i;
      }
   }

   // Computes the fundamental matrix between two frames given the transformation matrix
   void CollisionDetector::ComputeFundamentalMatrix(const cv::Mat & current_T_previous, const cv::Mat cam_K, cv::Mat &current_F_previous){
      // https://sourishghosh.com/2016/fundamental-matrix-from-camera-matrices/
      cv::Mat rotation = current_T_previous(cv::Range(0, 3), cv::Range(0, 3) );
      cv::Mat translation = (cv::Mat_<double>(3,1) << current_T_previous.at<double>(0,3),
              current_T_previous.at<double>(1,3),
              current_T_previous.at<double>(2,3));
      cv::Mat A = cam_K * rotation.t() * translation;
      cv::Mat cross_product_mat = (cv::Mat_<double>(3,3) << 0, -A.at<double>(2,0), A.at<double>(1,0),
              A.at<double>(2,0), 0, -A.at<double>(0,0),
              -A.at<double>(1,0), A.at<double>(0,0), 0);
      current_F_previous = (cam_K.inv()).t() * rotation * cam_K.t() * cross_product_mat;
      current_F_previous = current_F_previous/current_F_previous.at<double>(2,2);
   }

   void CollisionDetector::ComputeMatchesAllvsAll(const cv::Mat &descriptors_previous_i, const cv::Mat &descriptors_current_i, std::vector<cv::DMatch>& best_matches){
      // Performs the matching algorithm of all descriptors of one image against all descriptors of a second one
      if(FLAGS_detector_type == "SIFT" || FLAGS_detector_type == "SURF"){
         // Matching the desriptors using FLANN matcher
         std::vector<std::vector<cv::DMatch>> knn_matches;
         cv::Ptr<cv::DescriptorMatcher> matcher = DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
         matcher->knnMatch(descriptors_previous_i, descriptors_current_i, knn_matches,2);

         // Filtering of matches by Lowe's ratio test
         const float ratio_threshold = 0.7f;
         for(size_t i=0; i < knn_matches.size(); i++){
            if(knn_matches[i][0].distance < ratio_threshold * knn_matches[i][1].distance){
               best_matches.push_back(knn_matches[i][0]);
            }
         }
      }
   }

   void CollisionDetector::FilterMatchesByEpipolarConstrain(const std::vector <cv::KeyPoint> &keypoints_previous_i, const std::vector <cv::KeyPoint> &keypoints_current_i, 
                           const std::vector<cv::DMatch> &best_matches, const cv::Mat &F_matrix, const cv::Size2i &img_size, std::vector<cv::DMatch>& filtered_matches){
      // Check location of epipoles (if they are inside the image then it is a degenerate case of F_matrix)
      static int img_width = img_size.width;
      static int img_height = img_size.height;
      Eigen::Matrix3f F;
      cv::cv2eigen(F_matrix,F);
      Eigen::EigenSolver<Eigen::Matrix3f> eigen_matrix_right(F.transpose()*F),eigen_matrix_left(F*F.transpose());
      // Finding the index of the minimum eigenvalue
      float min_eigenvalue_right=1000;
      float min_eigenvalue_left=1000;
      int idx_eigenvalue_right, idx_eigenvalue_left;
      for(int i = 0; i < eigen_matrix_right.eigenvalues().rows(); i++){
         if(eigen_matrix_right.eigenvalues()[i].real() < min_eigenvalue_right){
            min_eigenvalue_right = eigen_matrix_right.eigenvalues()[i].real();
            idx_eigenvalue_right = i;
         }
         if(eigen_matrix_left.eigenvalues()[i].real() < min_eigenvalue_left){
            min_eigenvalue_left = eigen_matrix_left.eigenvalues()[i].real();
            idx_eigenvalue_left = i;
         }
      }

      if((idx_eigenvalue_right >= 0 && idx_eigenvalue_right <=2)&&(idx_eigenvalue_left >= 0 && idx_eigenvalue_left <=2)){
         // Check that the epipole in the current frame is not inside the image
         Eigen::Vector3cf epipole_right = eigen_matrix_right.eigenvectors().col(idx_eigenvalue_right)/eigen_matrix_right.eigenvectors().col(idx_eigenvalue_right)[2];
         // Check that the epipole in the previous frame is not inside the image
         Eigen::Vector3cf epipole_left = eigen_matrix_left.eigenvectors().col(idx_eigenvalue_left)/eigen_matrix_left.eigenvectors().col(idx_eigenvalue_left)[2];
         if((epipole_left[0].real() > 0 && epipole_left[0].real() <= img_width) && (epipole_left[1].real() > 0 && epipole_left[1].real() <= img_height) ||
         (epipole_right[0].real() > 0 && epipole_right[0].real() <= img_width) && (epipole_right[1].real() > 0 && epipole_right[1].real() <= img_height)){
            filtered_matches = best_matches;
            return;
         }
      }

      // Compute epipolar lines on the current image frame
      std::vector<cv::Point2f> points_previous_i, points_current_i;
      FromMatchesToVectorofPoints(keypoints_previous_i, keypoints_current_i, best_matches, // Transforms the matches into the correspondent input that
                                 points_previous_i,points_current_i);                      // the function cv::computeCorrespondEpilines requires
      std::vector<cv::Vec3f> lines_current_i;
      cv::computeCorrespondEpilines(points_previous_i, 1, F_matrix,lines_current_i);

      for(int l = 0; l < points_previous_i.size(); l++){
         double epipole_dist = DistancePointToLine(points_current_i[l],lines_current_i[l]);
         if(epipole_dist < FLAGS_int_epipolar_dst_thr) filtered_matches.push_back(best_matches[l]);
      }
   }

   // Filter matches using cosine similarity between rays
   void CollisionDetector::filterMatchesByAngleComparison(const std::vector<cv::KeyPoint> &keypoints_frame1, const std::vector<cv::KeyPoint> &keypoints_frame2, const cv::Mat &world_Tprevious_base,
                           const cv::Mat &world_Tcurrent_base, const cv::Mat cam_K, const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch>& filtered_matches){
      // Define the similarity that is used as threshold of matches
      const float angle_threshold_degree = 1;
      const float cosine_threshold = cos((1*3.1415)/180); 
      
      // Extract the position vector of the camera's centres
      cv::Mat world_camera_previous_position = (cv::Mat_<double>(3,1) << world_Tprevious_base.at<double>(0,3), world_Tprevious_base.at<double>(1,3), world_Tprevious_base.at<double>(2,3));
      cv::Mat world_camera_current_position = (cv::Mat_<double>(3,1) << world_Tcurrent_base.at<double>(0,3), world_Tcurrent_base.at<double>(1,3), world_Tcurrent_base.at<double>(2,3));
      // Extrinsic matrix as a 3x4 matrices
      cv::Mat world_T_previous_pose = world_Tprevious_base(cv::Range(0, 3), cv::Range(0, 4));
      cv::Mat world_T_current_pose = world_Tcurrent_base(cv::Range(0, 3), cv::Range(0, 4));

      // Going through all point correspondences //keypoints_previous_i[filtered_matches[pt_i].queryIdx].pt
      for(auto & match : matches){
         // Getting the (u,v) and (u',v') location of the point correspondences
         cv::Point2f feature_image_1 = keypoints_frame1[match.queryIdx].pt;
         cv::Point2f feature_image_2 = keypoints_frame2[match.trainIdx].pt;
         cv::Mat feat_img1_homogeneous = (cv::Mat_<double>(3,1) << feature_image_1.x, feature_image_1.y, 1);
         cv::Mat feat_img2_homogeneous = (cv::Mat_<double>(3,1) << feature_image_2.x, feature_image_2.y, 1);
         // Converting the points to world coordinates
         cv::Mat feat_img1_cam_coordinates = cam_K.inv() * feat_img1_homogeneous;
         cv::Mat feat_img2_cam_coordinates = cam_K.inv() * feat_img2_homogeneous;
         cv::Mat feat_img1_cam_coordinates_hom = (cv::Mat_<double>(4,1) << feat_img1_cam_coordinates.at<double>(0,0), feat_img1_cam_coordinates.at<double>(1,0), feat_img1_cam_coordinates.at<double>(2,0), 1);
         cv::Mat feat_img2_cam_coordinates_hom = (cv::Mat_<double>(4,1) << feat_img2_cam_coordinates.at<double>(0,0), feat_img2_cam_coordinates.at<double>(1,0), feat_img2_cam_coordinates.at<double>(2,0), 1);
         cv::Mat feat_img1_world = world_T_previous_pose * feat_img1_cam_coordinates_hom;
         cv::Mat feat_img2_world = world_T_current_pose * feat_img2_cam_coordinates_hom;
         // Compute rays direction
         cv::Mat feature_img1_ray = feat_img1_world(cv::Range(0, 3), cv::Range(0, 1)) - world_camera_previous_position;
         cv::Mat feature_img2_ray = feat_img2_world(cv::Range(0, 3), cv::Range(0, 1)) - world_camera_current_position;
         // Compute similarity
         float cos = feature_img1_ray.dot(feature_img2_ray)/(cv::norm(feature_img1_ray) * cv::norm(feature_img2_ray));

         //cout << "cos: " << cos << endl;

         if(cos < cosine_threshold){
            filtered_matches.push_back(match);
         }
      }
   }

   // Transform a vector of DMatch into a one of Point2d
   void CollisionDetector::FromMatchesToVectorofPoints(const std::vector<cv::KeyPoint> &keypoints_frame1, const std::vector<cv::KeyPoint> &keypoints_frame2,
                                    const std::vector<cv::DMatch> &matches, std::vector <cv::Point2f> &points_frame1,std::vector <cv::Point2f> &points_frame2){
      for (int p = 0; p < (int)matches.size(); p++) {
              points_frame1.push_back(keypoints_frame1[matches[p].queryIdx].pt);
              points_frame2.push_back(keypoints_frame2[matches[p].trainIdx].pt);
      }
   }

   // Computation of the orthogonal distance from a point to a line
   double CollisionDetector::DistancePointToLine( const cv::Point2f point, const cv::Vec3f epiline){
      return abs(epiline[0]*point.x + epiline[1]*point.y + epiline[2])/sqrtf(pow(epiline[0],2) + pow(epiline[1],2));
   }

   void CollisionDetector::TriangulatePoints(const cv::Mat &base_Tcam, const cv::Mat cam_K, const cv::Mat & current_T_previous, const std::vector <cv::KeyPoint> &keypoints_previous_i, const std::vector <cv::KeyPoint> &keypoints_current_i, 
                           const cv::Mat &descriptors_previous_i, const cv::Mat &descriptors_current_i, const std::vector<cv::DMatch>& filtered_matches, const cv::Mat &world_Tcurrent_base, bool b_to_world, 
                           std::vector<cv::Mat> &final_3d_pts, std::vector<double> &points_keypoints, cv::Mat &final_descriptors){
      // find the projective matrices of both frames
      cv::Mat P_previous, P_current;
      ComputeProjectionMatrices(cam_K,current_T_previous,P_previous,P_current);
      // Adapting the projection matrices for later use with OpenCV
      std::vector <cv::Mat> projection_matrices;
      std::vector <cv::Point2f> vec_filtered_matches_previous, vec_filtered_matches_current;
      projection_matrices.push_back(P_previous);
      projection_matrices.push_back(P_current);
      FromMatchesToVectorofPoints(keypoints_previous_i,keypoints_current_i,filtered_matches,vec_filtered_matches_previous,vec_filtered_matches_current);
      std::vector <cv::Mat> vec_filtered_points_2d;
      GetArrayOfPoints(vec_filtered_matches_previous, vec_filtered_matches_current, vec_filtered_points_2d);

      // Triangulation with OpenCV
      cv::Mat triangulated_points;
      int pixel_range = 1; // TODO: make it a parameter for modifying 
      triangulatePoints2Views(vec_filtered_points_2d,projection_matrices,pixel_range,triangulated_points);
      //cv::sfm::triangulatePoints(vec_filtered_points_2d,projection_matrices,triangulated_points);
      
      /// Check 3D points
      for(int pt_i = 0; pt_i < triangulated_points.size().width; pt_i++){
         cv::Mat pt_hom;
         cv::Mat pt_cam_previous, pt_cam_current;
         sfm::euclideanToHomogeneous(triangulated_points.col(pt_i),pt_hom);
         sfm::euclideanToHomogeneous(projection_matrices[0]*pt_hom,pt_cam_previous);
         sfm::euclideanToHomogeneous(projection_matrices[1]*pt_hom,pt_cam_current);
         //// Filter points behind the camera (-z)
         if(pt_cam_previous.at<double>(2, 0) < 0.0 || pt_cam_current.at<double>(2, 0) < 0.0) continue;

         //// Filter points with the reprojection error
         cv::Point2f feat_img_1 = keypoints_previous_i[filtered_matches[pt_i].queryIdx].pt;
         cv::Point2f feat_img_2 = keypoints_current_i[filtered_matches[pt_i].trainIdx].pt;

         pt_cam_previous = pt_cam_previous/pt_cam_previous.at<double>(2,0);
         pt_cam_current = pt_cam_current/pt_cam_current.at<double>(2,0);

         double errXprevious = pt_cam_previous.at<double>(0, 0) - cv::saturate_cast<double>(feat_img_1.x);
         double errYprevious = pt_cam_previous.at<double>(1, 0) - cv::saturate_cast<double>(feat_img_1.y);
         double errXcurrent = pt_cam_current.at<double>(0, 0) - cv::saturate_cast<double>(feat_img_2.x);
         double errYcurrent = pt_cam_current.at<double>(1, 0) - cv::saturate_cast<double>(feat_img_2.y);

         // If the error in x or y is bigger than 4 pixels we skip this point
         if(cv::sqrt(errXprevious * errXprevious + errYprevious * errYprevious) > 4.0) continue;
         if(cv::sqrt(errXcurrent * errXcurrent + errYcurrent * errYcurrent) > 4.0) continue;

         // Convert the point in camera frame previous to the current cam frame.
         pt_cam_current = base_Tcam * current_T_previous * pt_hom;
         // If the point is referrenced to the world
         if(b_to_world) pt_cam_current = world_Tcurrent_base * pt_cam_current;
         final_3d_pts.push_back(pt_cam_current);

         // Adding the descriptor of the point
         points_keypoints.insert(points_keypoints.end(), {cv::saturate_cast<double>(feat_img_2.x),cv::saturate_cast<double>(feat_img_2.y)});
         final_descriptors.push_back(descriptors_current_i.row(filtered_matches[pt_i].trainIdx));
      }
   }

   void CollisionDetector::homogeneousToEuclidean(cv::InputArray X_, cv::OutputArray x_){
      // src
      const cv::Mat X = X_.getMat();
      
      // dst
      x_.create(X.rows-1, X.cols, X.type());
      cv::Mat x = x_.getMat();
      
      // type
      if( X.depth() == CV_32F ){
         coldetector::homogeneousToEuclidean<float>(X,x);
      }
      else{
         coldetector::homogeneousToEuclidean<double>(X,x);
      }
   }

   // Performs the triangulation of 3D points from a set of 2d correspondences between two frames
   void CollisionDetector::triangulatePoints2Views(cv::InputArrayOfArrays _points2d, cv::InputArrayOfArrays _projection_matrices, 
                                                   const int &pixel_range,cv::OutputArray _points3d){
      // inputs
      size_t nviews = (unsigned) _points2d.total();
      size_t n_points;
      std::vector<Mat_<double>> points2d(nviews);
      std::vector<Matx34d> projection_matrices(nviews);
      {
         std::vector<Mat> points2d_tmp;
        _points2d.getMatVector(points2d_tmp);
        n_points = points2d_tmp[0].cols;

        std::vector<Mat> projection_matrices_tmp;
        _projection_matrices.getMatVector(projection_matrices_tmp);

        // Make sure the dimensions are right
        for(size_t i=0; i<nviews; ++i){
            CV_Assert(points2d_tmp[i].rows == 2 && points2d_tmp[i].cols == n_points);
            if (points2d_tmp[i].type() == CV_64F)
                points2d[i] = points2d_tmp[i];
            else
                points2d_tmp[i].convertTo(points2d[i], CV_64F);

            CV_Assert(projection_matrices_tmp[i].rows == 3 && projection_matrices_tmp[i].cols == 4);
            if (projection_matrices_tmp[i].type() == CV_64F)
              projection_matrices[i] = projection_matrices_tmp[i];
            else
              projection_matrices_tmp[i].convertTo(projection_matrices[i], CV_64F);
        }  
      }

      // output
      _points3d.create(3, n_points, CV_64F);
      cv::Mat points3d = _points3d.getMat();

      const Mat_<double> &xl = points2d[0], &xr = points2d[1];

        const Matx34d & Pl = projection_matrices[0];    // left matrix projection
        const Matx34d & Pr = projection_matrices[1];    // right matrix projection

        // triangulate
        for( unsigned i = 0; i < n_points; ++i )
        {
            Vec3d point3d;
            triangulateDLT( Vec2d(xl(0,i), xl(1,i)), Vec2d(xr(0,i), xr(1,i)), Pl, Pr, point3d );
            //Calculate the amount of uncertainty in the depth measurement
            float z_error;
            bool f_behind_cam = false;
            float triangulation_error_threshold = 0.02; // Error in the variance of the depth values expressed in m(?)
            
            if(z_error <= triangulation_error_threshold && f_behind_cam==false){
               for(char j=0; j<3; ++j)
                points3d.at<double>(j, i) = point3d[j];
            }
        }
   }

   void CollisionDetector::triangulateDLT(const cv::Vec2d &xl, const cv::Vec2d &xr, const cv::Matx34d &Pl, const cv::Matx34d &Pr, cv::Vec3d &point3d){
      cv::Matx44d design;
      for (int i = 0; i < 4; ++i)
      {
         design(0,i) = xl(0) * Pl(2,i) - Pl(0,i);
         design(1,i) = xl(1) * Pl(2,i) - Pl(1,i);
         design(2,i) = xr(0) * Pr(2,i) - Pr(0,i);
         design(3,i) = xr(1) * Pr(2,i) - Pr(1,i);
      }
      
      cv::Vec4d XHomogeneous;
      cv::SVD::solveZ(design, XHomogeneous);
      homogeneousToEuclidean(XHomogeneous, point3d);
   }

   void CollisionDetector::triangulatedPointUncertainty(const cv::Vec2d &xl, const cv::Vec2d &xr, const cv::Matx34d &Pl, const cv::Matx34d &Pr,const int pixel_range, bool &fbehind_camera, float &z_deviation){
      // Compute four 3D points separate by an amount of pixel_range (equidistant from the original xl-xr position)
      Vec3d point3d_right, point3d_bottom, point3d_left, point3d_top;
      triangulateDLT( Vec2d(xl(0)+1, xl(1)), Vec2d(xr(0)+1, xr(1)), Pl, Pr, point3d_right);
      triangulateDLT( Vec2d(xl(0), xl(1)+1), Vec2d(xr(0), xr(1)+1), Pl, Pr, point3d_bottom);
      triangulateDLT( Vec2d(xl(0)-1, xl(1)), Vec2d(xr(0)-1, xr(1)), Pl, Pr, point3d_left);
      triangulateDLT( Vec2d(xl(0), xl(1)-1), Vec2d(xr(0), xr(1)-1), Pl, Pr, point3d_top);

      Eigen::Vector4f z_measures = Eigen::Vector4f(point3d_right(2),point3d_bottom(2),point3d_left(2),point3d_top(2));
      if(z_measures.mean() <= 0){
         fbehind_camera = true;
      }
      z_deviation = std::abs(z_measures.maxCoeff() - z_measures.minCoeff());
   }
   

   // Creates the projection matrices out of the Rotation and translation parameters between two frames
   void CollisionDetector::ComputeProjectionMatrices(const cv::Mat cam_K, const cv::Mat &current_T_previous, cv::Mat &P_previous, cv::Mat &P_current){
      cv::Mat reference = cv::Mat::eye(3,4,CV_64F); // Located at frame 1 at the moment
      cv::Mat rotation = current_T_previous(cv::Range(0, 3), cv::Range(0, 3) );
      cv::Mat translation = (cv::Mat_<double>(3,1) << current_T_previous.at<double>(0,3),
              current_T_previous.at<double>(1,3),
              current_T_previous.at<double>(2,3));
      // Projective matrix comes from multiplying the intrinsics by the rotation & translation
      P_previous =  cam_K * reference;
      cv::sfm::projectionFromKRt(cam_K,rotation,translation,P_current);
   }

   // Transform points into a InputArrayOfArrays
   void CollisionDetector::GetArrayOfPoints(std::vector<cv::Point2f> &points_frame1, std::vector<cv::Point2f> &points_frame2, std::vector<cv::Mat> &array_of_points){
      cv::Mat points1Mat = (cv::Mat_<double>(2,1) << points_frame1[0].x, points_frame1[0].y);
      cv::Mat points2Mat = (cv::Mat_<double>(2,1) << points_frame2[0].x, points_frame2[0].y);
      for(int i=1; i < points_frame1.size(); i++){
          cv::Mat point1 = (cv::Mat_<double>(2,1) << points_frame1[i].x, points_frame1[i].y);
          cv::Mat point2 = (cv::Mat_<double>(2,1) << points_frame2[i].x, points_frame2[i].y);
          cv::hconcat(points1Mat,point1,points1Mat);
          cv::hconcat(points2Mat,point2,points2Mat);
      }
      array_of_points.push_back(points1Mat);
      array_of_points.push_back(points2Mat);
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
      {
         std::lock_guard<std::mutex> lock(mReceiveData);
         previous_imgs_frame_ = current_frame;
         previous_imgs_frame_.b_empty = false;
      }
	}

   void CollisionDetector::StopRequest(){
      {
        lock_guard<mutex> lock(mMutexStop);
        bFinishRequested_ = true;
      }
    }

    bool CollisionDetector::CheckifStop(){
        lock_guard<mutex> lock(mMutexStop);
        return bFinishRequested_;
    }
}