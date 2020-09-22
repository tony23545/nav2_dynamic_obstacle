import numpy as np 
import cv2
from scipy.optimize import linear_sum_assignment

from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.node import Node

class ObstacleClass:
    """wrap a kalman filter and extra information for one single obstacle

    Arrtibutes:
        position: 3d position of center point, numpy array with shape (3, 1)
        velocity: 3d velocity of center point, numpy array with shape (3, 1)
        kalman: cv2.KalmanFilter
        dying: count missing frames for this obstacle, if reach threshold, delete this obstacle
    """

    def __init__(self, obstacle_msg, idx, measurementNoiseCov, errorCovPost):
        '''Initialize with an Obstacle msg and an assigned id'''
        self.msg = obstacle_msg
        self.msg.id = idx
        position = np.array([[obstacle_msg.position.x, obstacle_msg.position.y, obstacle_msg.position.z]]).T # shape 3*1
        velocity = np.array([[obstacle_msg.velocity.x, obstacle_msg.velocity.y, obstacle_msg.velocity.z]]).T

        self.kalman = cv2.KalmanFilter(6,3) # 3d by default, 6d state space and 3d observation space
        self.kalman.measurementMatrix = np.array([[1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,1,0,0,0]], np.float32)
        self.kalman.measurementNoiseCov = np.diag(measurementNoiseCov).astype(np.float32)
        self.kalman.statePost = np.concatenate([position, velocity]).astype(np.float32)
        self.kalman.errorCovPost = np.diag(errorCovPost).astype(np.float32)
        
        self.dying = 0

    def predict(self, dt, a_noise):
        '''update F and Q matrices, call KalmanFilter.predict and store position and velocity'''

        # construct new transition matrix
        '''
        F = 1, 0, 0, dt, 0,  0
            0, 1, 0, 0,  dt, 0
            0, 0, 1, 0,  0,  dt
            0, 0, 0, 1,  0,  0
            0, 0, 0, 0,  1,  0
            0, 0, 0, 0,  0,  1
        '''
        F = np.eye(6).astype(np.float32)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # construct new process conv matrix
        '''assume constant velocity, and obstacle's acceleration has noise in x, y, z direction ax, ay, az
        Q = dt4*ax/4, 0,        0,        dt3*ax/2, 0,        0
            0,        dt4*ay/4, 0,        0,        dt3*ay/2, 0
            0,        0,        dt4*az/4, 0,        0,        dt3*az/2
            dt3*ax/2, 0,        0,        dt2*ax,   0,        0
            0,        dt3*ay/2, 0,        0,        dt2*ay,   0
            0,        0,        dt3*az/2, 0,        0,        dt2*az
        '''
        dt2 = dt**2
        dt3 = dt*dt2
        dt4 = dt2**2
        Q = np.array([[dt4*a_noise[0]/4, 0, 0, dt3*a_noise[0]/2, 0, 0], 
                      [0, dt4*a_noise[1]/4, 0, 0, dt3*a_noise[1]/2, 0],
                      [0, 0, dt4*a_noise[2]/4, 0, 0, dt3*a_noise[2]/2],
                      [dt3*a_noise[0]/2, 0, 0, dt2*a_noise[0], 0, 0],
                      [0, dt3*a_noise[1]/2, 0, 0, dt2*a_noise[1], 0],
                      [0, 0, dt3*a_noise[2]/2, 0, 0, dt2*a_noise[2]]]).astype(np.float32)

        self.kalman.transitionMatrix = F
        self.kalman.processNoiseCov = Q
        self.kalman.predict()

    def correct(self, detect_msg):
        '''extract position as measurement and update KalmanFilter'''
        measurement = np.array([[detect_msg.position.x, detect_msg.position.y, detect_msg.position.z]]).T.astype(np.float32)
        self.kalman.correct(measurement)
        self.msg.position.x = np.float(self.kalman.statePost[0][0])
        self.msg.position.y = np.float(self.kalman.statePost[1][0])
        self.msg.position.z = np.float(self.kalman.statePost[2][0])
        self.msg.velocity.x = np.float(self.kalman.statePost[3][0])
        self.msg.velocity.y = np.float(self.kalman.statePost[4][0])
        self.msg.velocity.z = np.float(self.kalman.statePost[5][0])

    def distance(self, other_msg):
        '''measurement distance between two obstacles, dy default it's Euler distance between centers
           you can extent the Obstacle msg to include more features like bounding box or radius and include in the distance function'''
        position = np.array([[self.msg.position.x, self.msg.position.y, self.msg.position.z]]).T
        other_position = np.array([[other_msg.position.x, other_msg.position.y, other_msg.position.z]]).T
        return np.linalg.norm(position - other_position)

class KFHungarianTracker(Node):
    '''Use Kalman Fiter and Hungarian algorithm to track multiple dynamic obstacles

    Use Hungarian algorithm to match presenting obstacles with new detection and maintain a kalman filter for each obstacle.
    spawn ObstacleClass when new obstacles come and delete when they disappear for certain number of frames

    Attributes:
        obstacle_list: a list of ObstacleClass that currently present in the scene
        max_id: the maximum id assigned 
        sec, nanosec: timing from sensor msg
        detection_sub: subscrib detection result from detection node
        tracker_obstacle_pub: publish tracking obstacles with ObstacleArray
        tracker_pose_pub: publish tracking obstacles with PoseArray, for rviz visualization
    '''

    def __init__(self):
        '''initialize attributes and setup subscriber and publisher'''

        super().__init__('kf_hungarian_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('a_noise', None),
                ('death_threshold', None),
                ('measurementNoiseCov', None),
                ('errorCovPost', None)
            ])
        self.death_threshold = self.get_parameter("death_threshold")._value
        self.measurementNoiseCov = self.get_parameter("measurementNoiseCov")._value
        self.errorCovPost = self.get_parameter("errorCovPost")._value
        self.a_noise = self.get_parameter("a_noise")._value

        self.obstacle_list = []
        self.max_id = 0
        self.sec = 0
        self.nanosec = 0

        # subscribe to detector 
        self.detection_sub = self.create_subscription(
            ObstacleArray,
            "detection",
            self.callback,
            10)

        # publisher for tracking result
        self.tracker_obstacle_pub = self.create_publisher(ObstacleArray, 'tracking', 10)
        self.tracker_pose_pub = self.create_publisher(MarkerArray, 'tracking_pose_array', 10)

    def callback(self, msg):
        '''callback function for detection result'''

        # update delta time
        dt = (msg.header.stamp.sec - self.sec) + (msg.header.stamp.nanosec - self.nanosec) / 1e9
        self.sec = msg.header.stamp.sec
        self.nanosec = msg.header.stamp.nanosec

        # get detection
        detections = msg.obstacles
        num_of_detect = len(detections)
        num_of_obstacle = len(self.obstacle_list)

        # kalman predict
        for obj in self.obstacle_list:
            obj.predict(dt, self.a_noise)

        # hungarian matching
        cost = np.zeros((num_of_obstacle, num_of_detect))
        for i in range(num_of_obstacle):
            for j in range(num_of_detect):
                cost[i, j] = self.obstacle_list[i].distance(detections[j])
        obs_ind, det_ind = linear_sum_assignment(cost)

        # kalman update
        for o, d in zip(obs_ind, det_ind):
            self.obstacle_list[o].correct(detections[d])

        # birth of new detection obstacles and death of disappear obstacle
        if num_of_obstacle <= num_of_detect:
            self.birth(det_ind, num_of_detect, detections)
        else:
            self.death(obs_ind, num_of_obstacle)

        # construct ObstacleArray
        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        track_list = []
        for obs in self.obstacle_list:
            track_list.append(obs.msg)
        obstacle_array.obstacles = track_list
        self.tracker_obstacle_pub.publish(obstacle_array)

        # rviz visualization
        marker_array = MarkerArray()
        marker_list = []
        for obs in self.obstacle_list:
            marker = Marker()
            marker.header = msg.header
            marker.id = obs.msg.id
            marker.type = 1
            marker.action = 0
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.pose.position = obs.msg.position
            angle = np.arctan2(obs.msg.velocity.y, obs.msg.velocity.x)
            marker.pose.orientation.z = np.float(np.sin(angle / 2))
            marker.pose.orientation.w = np.float(np.cos(angle / 2))
            marker.scale = obs.msg.scale
            marker_list.append(marker)
        marker_array.markers = marker_list
        self.tracker_pose_pub.publish(marker_array)

    def birth(self, det_ind, num_of_detect, detections):
        '''generate new ObstacleClass for detections that do not match any in current obstacle list'''
        for det in range(num_of_detect):
            if det not in det_ind:
                self.obstacle_list.append(ObstacleClass(detections[det], self.max_id, self.measurementNoiseCov, self.errorCovPost))
                self.max_id =  self.max_id  + 1

    def death(self, obj_ind, num_of_obstacle):
        '''count obstacles' missing frames and delete when reach threshold'''
        new_object_list = []
        for obs in range(num_of_obstacle):
            if obs not in obj_ind:
                self.obstacle_list[obs].dying += 1
            else:
                self.obstacle_list[obs].dying = 0

            if self.obstacle_list[obs].dying < self.death_threshold:
                new_object_list.append(self.obstacle_list[obs])
        self.obstacle_list = new_object_list

def main(args=None):
    rclpy.init(args=args)

    node = KFHungarianTracker()
    node.get_logger().info("start spining tracker node...")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
