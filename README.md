# Nav2 Dynamic Obstacle

This module implement both the `perception` and `tracking` for 3D mutil-object tracking to detect and track dynamic obstacles around the robot.  

A minimum representation of a dynamic obstacle includes position and velocity. More features like bounding box or raduis could be included to improve the tracking performance. 

In `detectron2_detector`, [Detectron2](https://github.com/facebookresearch/detectron2) is applied on RGB image to extract mask for target object and then RGBD image is used to estimate 3D position.

In `detectron2_detector`, Kalman filter is used to tracking dynamic obstacles in 3D space and Hungarian algorithm is used to associate obstacles and detection results. 
