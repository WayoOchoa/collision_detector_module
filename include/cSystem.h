#pragma once

#include <iostream>
#include <string>

// OpenCv library
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

namespace coldetector{
    class cSystem{
        private:
            // Number of cameras
            int nrCams_;
            // Camera calibration matrices
            std::vector<cv::Matx44d> M_c_; // Extrinsics
            std::vector<cv::Mat_<double>> K_c_; // Intrinsics
        
        public:
            /**
             * Create a new cSystem object.
             * @brief Default constructor.
             * @see cSystem(const std::string& path2calib)
             * @see
            */
            cSystem(): nrCams_(0){};
            /**
             * @brief Default destructor of a cSystem object.
            */
            ~cSystem(){};
            /**
             * Construct a new cSystem object from a configuration file.
             * @brief Constructor.
             * @param path2calib Path to configuration files.
            */
            cSystem(const std::string& path2calib);
    
            /**
             * @brief Loads the cameras calibration parameters from the configuration
             * files.
             * @param path2calib Path where the calibration parameters files are saved in
             * the system.
            */
            void LoadMCS(const std::string path2calib);
            /**
             * @brief Set the camera system configuration.
             * @param numCams Number of cameras that compose the system.
             * @param cayley_calibration_params Vector of matrices containing the 6 cayley parameters for each camera.
             * @param intrinsic_params Vector of matrices containing the instrinsic parameters for each camera.
            */
            void setSystemConfig(int numCams, std::vector<cv::Matx61d> cayley_calibration_params, std::vector<cv::Matx41d> intrinsic_params);
            /**
             * Gets the instrinsic calibration matrix from the camera c
             * @param cam The camera number identifier. 
             * @return K_c A 3x3 Intrinsic calibration matrix.
            */
            cv::Mat getK_c(int cam);
            /**
             * Gets the extrinsic calibration matrix from the camera c.
             * @param cam The camera number identifier. 
             * @return M_c A 4x4 Extrinsics calibration matrix.
            */
            cv::Matx44d getM_c(int cam);
            /**
             * Gets the number of cameras in the system
             * @return nCams Number of cameras.
            */
            int get_nrCams();
    
            /**
             * 6x1 minimal homogeneous transformation vector to homogeneous 4x4 transformation mattrix
             * @param c 6x1 Cayley parameters and translation
             * @return T 4x4 homogeneous transformation matrix
            */
            template<typename T>
            cv::Matx<T,4,4> cayley2hom(const cv::Matx<T,6,1>& cayleyRep){
                cv::Matx<T, 3, 1> cayleyR(cayleyRep(0, 0), cayleyRep(1, 0), cayleyRep(2, 0));
                cv::Matx<T, 3, 3> R = cayley2rot(cayleyR);
    
                cv::Matx<T, 4, 4> homM(
                    R(0, 0), R(0, 1), R(0, 2), cayleyRep(3, 0),
                    R(1, 0), R(1, 1), R(1, 2), cayleyRep(4, 0),
                    R(2, 0), R(2, 1), R(2, 2), cayleyRep(5, 0),
                    T(0), T(0), T(0), T(1));
                
                return homM;
            }
            /**
             * Cayley representation to 3x3 rotation matrix
             * @param cayParamIn Cayley parameters.
             * @return R rotation matrix.
            */
            template<typename T>
    	    cv::Matx<T, 3, 3> cayley2rot(const cv::Matx<T, 3, 1>& cayParamIn)
    	    {
                cv::Matx<T, 3, 3>  R = cv::Matx<T, 3, 3>::eye();
                
                T c1 = cayParamIn(0, 0);
    		    T c2 = cayParamIn(1, 0);
    		    T c3 = cayParamIn(2, 0);
    
    		    T c1sqr = c1*c1;
    		    T c2sqr = c2*c2;
    		    T c3sqr = c3*c3;
    
    		    T scale = T(1) + c1sqr + c2sqr + c3sqr;
    
    		    R(0, 0) = 1 + c1sqr - c2sqr - c3sqr;
    		    R(0, 1) = 2 * (c1*c2 - c3);
    		    R(0, 2) = 2 * (c1*c3 + c2);
    		    R(1, 0) = 2 * (c1*c2 + c3);
    		    R(1, 1) = 1 - c1sqr + c2sqr - c3sqr;
    		    R(1, 2) = 2 * (c2*c3 - c1);
    		    R(2, 0) = 2 * (c1*c3 - c2);
    		    R(2, 1) = 2 * (c2*c3 + c1);
    		    R(2, 2) = 1 - c1sqr - c2sqr + c3sqr;
    
    		    R = (1 / scale) * R;
    
    		    return R;
    	    }
            /**
             * Create a intrinsic camera matrix K for the given parameters
             * @param fx Focal length in x.
             * @param fy Focal length in y.
             * @param px Principal point coordinate in x.
             * @param py Principal point coordinate in y.
            */
            cv::Mat_<double> setPinholeCameraMatrix(const double fx,const double fy, const double px, const double py)
    		{
    			cv::Mat_<double> pinhole_K = (cv::Mat_<double>(3,3) << fx, 0, px, 0, fy, py, 0,0,1.0);
                return pinhole_K;
    		}
    };

}