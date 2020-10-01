import numpy as np 
from scipy.optimize import linear_sum_assignment

from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from visualization_msgs.msg import Marker, MarkerArray

import rclpy
from rclpy.node import Node
import colorsys
from kf_hungarian_tracker.obstacle_class import ObstacleClass

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
                ('a_noise', [2., 2., 0.5]),
                ('top_down', False),
                ('death_threshold', 3),
                ('measurementNoiseCov', [1., 1., 1.]),
                ('errorCovPost', [1., 1., 1., 10., 10., 10.]),
                ('vel_filter', 0.2),
                ('cost_filter', 1.0)
            ])
        self.death_threshold = self.get_parameter("death_threshold")._value
        self.measurementNoiseCov = self.get_parameter("measurementNoiseCov")._value
        self.errorCovPost = self.get_parameter("errorCovPost")._value
        self.a_noise = self.get_parameter("a_noise")._value
        self.vel_filter = self.get_parameter("vel_filter")._value
        self.top_down = self.get_parameter("top_down")._value
        self.cost_filter = self.get_parameter("cost_filter")._value

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
        self.tracker_marker_pub = self.create_publisher(MarkerArray, 'marker', 10)

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
            obj.predict(dt)

        # hungarian matching
        cost = np.zeros((num_of_obstacle, num_of_detect))
        for i in range(num_of_obstacle):
            for j in range(num_of_detect):
                cost[i, j] = self.obstacle_list[i].distance(detections[j])
        obs_ind, det_ind = linear_sum_assignment(cost)

        # filter assignment according to cost
        new_obs_ind = []
        new_det_ind = []
        for o, d in zip(obs_ind, det_ind):
            if cost[o, d] < self.cost_filter:
                new_obs_ind.append(o)
                new_det_ind.append(d)
        obs_ind = new_obs_ind
        det_ind = new_det_ind

        # kalman update
        for o, d in zip(obs_ind, det_ind):
            self.obstacle_list[o].correct(detections[d])

        # birth of new detection obstacles and death of disappear obstacle
        self.birth(det_ind, num_of_detect, detections)
        dead_object_list = self.death(obs_ind, num_of_obstacle)

        # check if there is subscirbers
        if self.tracker_obstacle_pub.get_subscription_count() == 0 and self.tracker_marker_pub.get_subscription_count() == 0:
            return

        # construct ObstacleArray
        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        track_list = []
        for obs in self.obstacle_list:
            # do not publish obstacles with low speed
            obs_vel = np.linalg.norm(np.array([obs.msg.velocity.x, obs.msg.velocity.y, obs.msg.velocity.z]))
            if obs_vel > self.vel_filter:
                track_list.append(obs.msg)
        obstacle_array.obstacles = track_list
        self.tracker_obstacle_pub.publish(obstacle_array)

        # rviz visualization
        marker_array = MarkerArray()
        marker_list = []
        # add current active obstacles
        for obs in self.obstacle_list:
            (r, g, b) = colorsys.hsv_to_rgb(obs.msg.id * 30. / 360., 1., 1.) # encode id with rgb color
            # make a cube 
            marker = Marker()
            marker.header = msg.header
            marker.id = obs.msg.id
            marker.type = 1 # CUBE
            marker.action = 0
            marker.color.a = 0.5
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.pose.position = obs.msg.position
            angle = np.arctan2(obs.msg.velocity.y, obs.msg.velocity.x)
            marker.pose.orientation.z = np.float(np.sin(angle / 2))
            marker.pose.orientation.w = np.float(np.cos(angle / 2))
            marker.scale = obs.msg.size
            marker_list.append(marker)
            # make an arrow
            arrow = Marker()
            arrow.header = msg.header
            arrow.id = 255 - obs.msg.id
            arrow.type = 0
            arrow.action = 0
            arrow.color.a = 1.0
            arrow.color.r = r
            arrow.color.g = g
            arrow.color.b = b
            arrow.pose.position = obs.msg.position
            arrow.pose.orientation.z = np.float(np.sin(angle / 2))
            arrow.pose.orientation.w = np.float(np.cos(angle / 2))
            arrow.scale.x = np.linalg.norm([obs.msg.velocity.x, obs.msg.velocity.y, obs.msg.velocity.z])
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            marker_list.append(arrow)
        # add dead obstacles to delete in rviz
        for idx in dead_object_list:
            marker = Marker()
            marker.header = msg.header
            marker.id = idx
            marker.action = 2 # delete
            arrow = Marker()
            arrow.header = msg.header
            arrow.id = 255 - idx
            arrow.action = 2
            marker_list.append(marker)
            marker_list.append(arrow)

        marker_array.markers = marker_list
        self.tracker_marker_pub.publish(marker_array)

    def birth(self, det_ind, num_of_detect, detections):
        '''generate new ObstacleClass for detections that do not match any in current obstacle list'''
        for det in range(num_of_detect):
            if det not in det_ind:
                self.obstacle_list.append(ObstacleClass(detections[det], self.max_id, self.top_down, self.measurementNoiseCov, self.errorCovPost, self.a_noise))
                self.max_id =  self.max_id  + 1

    def death(self, obj_ind, num_of_obstacle):
        '''count obstacles' missing frames and delete when reach threshold'''
        new_object_list = []
        dead_object_list = []
        # for previous obstacles
        for obs in range(num_of_obstacle):
            if obs not in obj_ind:
                self.obstacle_list[obs].dying += 1
            else:
                self.obstacle_list[obs].dying = 0

            if self.obstacle_list[obs].dying < self.death_threshold:
                new_object_list.append(self.obstacle_list[obs])
            else:
                dead_object_list.append(self.obstacle_list[obs].msg.id)
        
        # add newly born obstacles
        for obs in range(num_of_obstacle, len(self.obstacle_list)):
            new_object_list.append(self.obstacle_list[obs])

        self.obstacle_list = new_object_list
        return dead_object_list

def main(args=None):
    rclpy.init(args=args)

    node = KFHungarianTracker()
    node.get_logger().info("start spining tracker node...")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
