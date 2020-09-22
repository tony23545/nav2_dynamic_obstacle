from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import _init_paths
import sys
# CENTERTRACK_PATH = "/home/tony/CenterTrack/src/lib"
# sys.path.insert(0, CENTERTRACK_PATH)

from detector import Detector
from opts import opts

import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
import cv2
import numpy as np

image_ext = ['jpg', 'jpeg', 'png', 'webp']
video_ext = ['mp4', 'mov', 'avi', 'mkv']
time_stats = ['tot', 'load', 'pre', 'net', 'dec', 'post', 'merge', 'display']

#MODEL_PATH = "../models/coco_pose.pth"
#TASK = 'tracking' # or 'tracking,multi_pose' for pose tracking and 'tracking,ddd' for monocular 3d tracking
#opt = opts().init('{} --load_model {}'.format(TASK, MODEL_PATH).split(' '))
opt = opts().init()
opt.debug = max(opt.debug, 1)
detector = Detector(opt)

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/kitti/camera_color_left/image_raw',
            self.callback,
            30)
        self.cnt = 0

    def callback(self, msg):
        print("get it")
        img = np.array(msg.data)
        img = img.reshape((480, 640, 3))
        img = np.flip(img, 2)
        self.cnt += 1
        
        #print(ret)
        cv2.imshow("input", img)

        ret = detector.run(img)

        # log run time
        time_str = 'frame {} |'.format(self.cnt)
        for stat in time_stats:
            time_str = time_str + '{} {:.3f}s |'.format(stat, ret[stat])
        print(time_str)

        print(ret['results'])
        if len(ret['results']) > 0:
            import IPython
            IPython.embed()


rclpy.init(args = None)
subs = MinimalSubscriber()
print("start spining node...")
rclpy.spin(subs)

subs.destroy_node()
rclpy.shutdown()

# for img in images:
#   ret = detector.run(img)['results']

