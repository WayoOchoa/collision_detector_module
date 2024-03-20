# Collision Detection and Avoidance for AUV
Library that implements functions for risk assessment of obstacles (in the form of a 3D point cloud) in underwater environments. In contains the necessary functions for feature detection, feature matching, 3D points triangulation, and risk assessment; based on our paper [Collision Detection and Avoidance for Underwater Vehicles Using Omnidirectional Vision](https://doi.org/10.3390/s22145354).

## Dependencies
The library was implemented in Ubuntu 18.04 and using ROS melodic. To build the project the following dependencies are also required:
 - [ROS](https://wiki.ros.org/Installation/Ubuntu)
 - Glog library
    ```console
    sudo apt install libgoogle-glog-dev
    ```
 - Gflags library
    ```console
    sudo apt install libgflags-dev
    ```
 - [OpenCV4.5](https://gist.github.com/kleysonr/c0752306bb6c021a1ff3c448996636ee) with contribution modules
 - PCL Library
    ```console
    sudo apt install libpcl-dev
    ```
 - Boost Library
    ```console
    sudo apt install libboost-all-dev
    ```
 - cv_bridge package for compatibility of ROS images and OpenCV4
    - In your catkin workspace src:
    ```console
    git clone https://github.com/fizyr-forks/vision_opencv/tree/opencv4
    cd vision_opencv
    git checkout opencv4
    ```
    - Follow this [link](https://stackoverflow.com/questions/63345411/ros-question-how-to-compile-custom-cv-bridge-with-opencv4-correctly) for further issues.

## Installation
TODO

## Run
TODO

## Documentation
See the library documentation for further info [<documentation>](https://wayoochoa.github.io/collision_detector_module/)
