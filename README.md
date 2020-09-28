# Nav2 Dynamic Obstacle

This module implements a highly extendable `detection-tracking` framework for robotic navigation. A basic detection pkg `detectron2_detector` and a tracking pkg `kf_hungarian_tracker` are provided. 

A minimum representation of a dynamic obstacle includes position, velocity and bounding box. More features could be included to satisfy special need. 

In `detectron2_detector`, [Detectron2](https://github.com/facebookresearch/detectron2) is applied on RGB image to extract mask for target object and then RGBD image is used to estimate 3D position.

In `kf_hungarian_tracker`, Kalman filter is used to tracking dynamic obstacles in 3D space and Hungarian algorithm is used to associate obstacles and detection results. 

## Environment
- Ubuntu 20.04
- ROS2 foxy
- python3.8
