#pragma once

#include <iostream>

// OpenCV libraries
#include <opencv2/opencv.hpp>

class MultiFrame{
    private:
        bool foo; // Temporal variable

    public:
        /**
         * Create a new MultiFrame object.
         * @brief Default constructor.
         * @see Multiframe(const std::vector<cv::Mat>& images, const double &timestamp)
        */
        MultiFrame(){};
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
        MultiFrame(const std::vector<cv::Mat>& images, const double &timestamp);

        /// Class data members
        // Images
        std::vector<cv::Mat> images_;
        // Frame timestamp
        double timestamp_;
};