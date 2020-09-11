import numpy as np 
import cv2
from scipy.optimize import linear_sum_assignment

from nav2_dynamic_msgs.msg import ObjectCircle, ObjectCircleArray
from geometry_msgs.msg import Pose, PoseArray

import rclpy
from rclpy.node import Node

class Object:
    def __init__(self, pos, idx, r, dt = 0.33):
        self.pos = pos.reshape((2, 1))
        self.kalman = cv2.KalmanFilter(4,2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0],[0,1,0,0]],np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,dt,0],[0,1,0,dt],[0,0,1,0],[0,0,0,1]],np.float32)
        self.kalman.processNoiseCov = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],np.float32) * 2.0
        self.kalman.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 1.0
        self.kalman.statePost = np.concatenate([pos, np.zeros(2)]).astype(np.float32).reshape(4, 1)
        '''
        # TODO 2nd order
        self.pos = pos.reshape((2, 1))
        self.kalman = cv2.KalmanFilter(6,2)
        self.kalman.measurementMatrix = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0]],np.float32)
        self.kalman.transitionMatrix = np.array([[1,0,dt,0,0.5*dt*dt,0],[0,1,0,dt,0,0.5*dt*dt],[0,0,1,0,dt,0],[0,0,0,1,0,dt],[0,0,0,0,1,0],[0,0,0,0,0,1]],np.float32)
        self.kalman.processNoiseCov = np.eye(6, dtype = np.float32) * 3.0
        self.kalman.measurementNoiseCov = np.array([[1, 0], [0, 1]], np.float32) * 2.0
        self.kalman.statePost = np.concatenate([pos, np.zeros(4)]).astype(np.float32).reshape(6, 1)
        '''
        self.dying = 0
        self.hit = False
        self.id = idx
        self.r = r
        self.vel = np.zeros(2)

    def predict(self):
        self.kalman.predict()
        self.pos = self.kalman.statePre[:2]

    def correct(self, measurement):
        self.kalman.correct(measurement)
        self.pos = self.kalman.statePost[:2]
        self.vel = self.kalman.statePost[2:4]

class KFHungarianTracker(Node):
    def __init__(self):
        super().__init__('kf_hungarian_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detector_topic', None),
            ])

        self.object_list = []
        self.max_id = 0

        # subscribe to detector 
        self.subscription = self.create_subscription(
            ObjectCircleArray,
            self.get_parameter("detector_topic")._value,
            self.callback,
            1)

        self.tracker_object_pub = self.create_publisher(ObjectCircleArray, 'tracker_object_array', 10)
        self.tracker_pose_pub = self.create_publisher(PoseArray, 'tracker_pose_array', 10)

    def callback(self, msg):
        self.get_logger().debug("tracker update...")
        detections = msg.objects
        detect_list = []
        radius_list = []
        for det in detections:
            detect_list.append(np.array([det.x, det.y]))
            radius_list.append(det.r)
        num_of_object = len(self.object_list)
        num_of_detect = len(detect_list)

        for obj in self.object_list:
            obj.predict()

        cost = np.zeros((num_of_object, num_of_detect))

        for i in range(num_of_object):
            for j in range(num_of_detect):
                cost[i, j] = np.linalg.norm(self.object_list[i].pos.reshape(2) - detect_list[j])

        obj_ind, det_ind = linear_sum_assignment(cost)

        for o, d in zip(obj_ind, det_ind):
            self.object_list[o].correct(detect_list[d].astype(np.float32).reshape(2, 1))

        if num_of_object <= num_of_detect: # there are new detection
            self.birth(det_ind, num_of_detect, detect_list, radius_list)
            #TODO filter out high cost
        else:
            self.death(obj_ind, num_of_object)

        # construct ObjectCircleArray
        object_array = ObjectCircleArray()
        object_array.header = msg.header
        track_list = []
        for obj in self.object_list:
            ObjectMsg = ObjectCircle()
            ObjectMsg.x = np.float(obj.pos[0])
            ObjectMsg.y = np.float(obj.pos[1])
            ObjectMsg.vx = np.float(obj.vel[0])
            ObjectMsg.vy = np.float(obj.vel[1])
            ObjectMsg.r = obj.r
            ObjectMsg.id = obj.id
            track_list.append(ObjectMsg)
        object_array.objects = track_list
        object_array.object_num = len(track_list)
        self.tracker_object_pub.publish(object_array)

        # construct PoseArray
        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_list = []
        for obj in self.object_list:
            p = Pose()
            p.position.x = np.float(obj.pos[0])
            p.position.y = np.float(obj.pos[1])
            angle = np.arctan2(obj.vel[1], obj.vel[0])
            p.orientation.z = np.float(np.sin(angle / 2))
            p.orientation.w = np.float(np.cos(angle / 2))
            pose_list.append(p)
        pose_array.poses = pose_list
        self.tracker_pose_pub.publish(pose_array)

    def birth(self, det_ind, num_of_detect, detect_list, radius_list):
        for det in range(num_of_detect):
            if det not in det_ind:
                self.object_list.append(Object(detect_list[det], self.max_id, radius_list[det]))
                self.max_id += 1

    def death(self, obj_ind, num_of_object):
        new_object_list = []
        for obj in range(num_of_object):
            if obj not in obj_ind:
                self.object_list[obj].dying += 1
            else:
                self.object_list[obj].dying = 0

            if self.object_list[obj].dying < 2:
                new_object_list.append(self.object_list[obj])
        self.object_list = new_object_list

def main(args=None):
    rclpy.init(args=args)

    node = KFHungarianTracker()
    node.get_logger().info("running tracker node...")

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
