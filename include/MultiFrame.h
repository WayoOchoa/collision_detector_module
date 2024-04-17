#pragma once

#include <iostream>

// OpenCV libraries
#include <opencv2/opencv.hpp>
#include <opencv2/sfm.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

namespace coldetector{
    class MultiFrame{
        private:
            bool foo; // Temporal variable

        public:
            /**
             * Create a new MultiFrame object.
             * @brief Default constructor.
             * @see Multiframe(const std::vector<cv::Mat>& images, const double &timestamp)
            */
            MultiFrame():b_empty(true){};
            /**
             * @brief Default destructor of a MultiFrame object.
            */
            ~MultiFrame(){};
            /**
             * Create a new MultiFrame object from camera images.
             * @brief Constructor.
             * @param images Read images from the multi-camera system.
             * @param timestamp Associated timestamp at which the reading was done.
            */
            MultiFrame(const std::vector<cv::Mat>& images, const cv::Mat &cam_pose, const double &timestamp, const int &numCams);

            /// Class data members
            // Number of camera images
            int nrCams;
            // Images vector
            std::vector<cv::Mat> images_;
            // Frame timestamp
            double timestamp_;
            // Camera position at the time the frame was acquired w.r.t the world
            cv::Mat world_Tcamera_base_;
            // Camera images features and descriptors
            std::vector<std::vector<cv::KeyPoint>> frame_keypoints_;
            std::vector<cv::Mat> frame_descriptors_;
            // Flag to check if the frame is empty
            bool b_empty;
    };
}