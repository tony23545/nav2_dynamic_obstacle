import detectron2
from detectron2.utils.logger import setup_logger
setup_logger()

# import from common libraries
import numpy as np 
import os, json, cv2, random

# import some common detectron2 utilities
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog, DatasetCatalog

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, PointCloud2 
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Pose, Point

class Detectron2Detector(Node):
    '''use Detectron2 to detect object masks from 2D image and estimate 3D position with Pointcloud2 data
    '''
    def __init__(self):
        super().__init__('detectron_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detectron_config_file', None),
                ('detectron_score_thresh', 0.5),
                ('pointcloud2_topic', None),
                ('pc_downsample_factor', 1),
                ('min_mask', 100),
                ('categories', [])
            ])
        self.pc_downsample_factor = int(self.get_parameter("pc_downsample_factor")._value)
        self.min_mask = self.get_parameter("min_mask")._value
        self.categories = self.get_parameter("categories")._value

        # setup detectron model
        self.cfg = get_cfg()
        config_file = self.get_parameter("detectron_config_file")._value
        self.cfg.merge_from_file(model_zoo.get_config_file(config_file))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = self.get_parameter("detectron_score_thresh")._value
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(config_file)
        self.predictor = DefaultPredictor(self.cfg)

        # subscribe to sensor 
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("pointcloud2_topic")._value,
            self.callback,
            1)

        # setup publisher
        self.detect_obj_pub = self.create_publisher(ObstacleArray, 'detection', 10)
        self.detect_img_pub = self.create_publisher(Image, 'image', 10)

        self.count = -1

    def outlier_filter(self, x, z, idx):
        '''simple outlier filter, assume Gaussian distribution and drop points with low probability (too far away from center)'''
        x_mean = np.mean(x)
        x_var = np.var(x)
        z_mean = np.mean(z)
        z_var = np.var(z)
        # probability under Gaussian distribution
        gaussian_kernel = np.exp(-0.5 * (np.power(x-x_mean, 2) / x_var + np.power(z-z_mean, 2) / z_var)) / (2 * np.pi * np.sqrt(x_var * z_var))
        return idx[gaussian_kernel > 0.5]

    def callback(self, msg):
        # extract data from msg
        height = msg.height
        width = msg.width
        points = np.array(msg.data, dtype = 'uint8')

        # decode rgb image
        rgb_offset = msg.fields[3].offset
        point_step = msg.point_step
        r = points[rgb_offset::point_step]
        g = points[(rgb_offset+1)::point_step]
        b = points[(rgb_offset+2)::point_step]
        img = np.concatenate([r[:, None], g[:, None], b[:, None]], axis = -1)
        img = img.reshape((height, width, 3))

        # decode point cloud data
        if msg.fields[0].datatype < 3:
            byte = 1
        elif msg.fields[0].datatype < 5:
            byte = 2
        elif msg.fields[0].datatype < 8:
            byte = 4
        else:
            byte = 8
        points = points.view('<f' + str(byte))
        x = points[0::int(self.pc_downsample_factor * point_step / byte)]
        y = points[1::int(self.pc_downsample_factor * point_step / byte)]
        z = points[2::int(self.pc_downsample_factor * point_step / byte)]

        # call detectron2 model
        outputs = self.predictor(img)

        # map mask to point cloud data
        color = np.zeros_like(x, dtype = 'uint8')
        num_classes = outputs['instances'].pred_classes.shape[0]
        if num_classes == 0:
            self.detect_obj_pub.publish(ObstacleArray())
            return

        masks = outputs["instances"].pred_masks.cpu().numpy().astype('uint8').reshape((num_classes, -1))[:, ::self.pc_downsample_factor]

        # estimate 3D position with simple averaging of obstacle's points
        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        detections = []
        for i in range(num_classes):
            if outputs["instances"].pred_classes[i] in self.categories:
                idx = np.where(masks[i])[0]
                idx = self.outlier_filter(x[idx], z[idx], idx)
                if idx.shape[0] < self.min_mask:
                    continue
                obstacle_msg = Obstacle()
                # pointcloud2 data has a different coordinate, swap y and z
                obstacle_msg.position.x = np.float(x[idx].mean())
                obstacle_msg.position.y = np.float(z[idx].mean())
                obstacle_msg.position.z = np.float(y[idx].mean())
                obstacle_msg.scale.x = np.float(x[idx].max() - x[idx].min())
                obstacle_msg.scale.y = np.float(z[idx].max() - z[idx].min())
                obstacle_msg.scale.z = np.float(y[idx].max() - y[idx].min())
                detections.append(obstacle_msg)

        # publishe detection result 
        obstacle_array.obstacles = detections
        self.detect_obj_pub.publish(obstacle_array)

        # visualize detection with detectron API
        v = Visualizer(img[:, :, ::-1], MetadataCatalog.get(self.cfg.DATASETS.TRAIN[0]), scale=1)
        out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
        out_img = out.get_image()[:, :, ::-1]
        out_img_msg = Image()
        out_img_msg.header = msg.header
        out_img_msg.height = height
        out_img_msg.width = width
        out_img_msg.encoding = 'rgb8'
        out_img_msg.step = 3 * width
        out_img_msg.data = out_img.flatten().tolist()
        self.detect_img_pub.publish(out_img_msg)
        
def main():
    rclpy.init(args = None)
    node = Detectron2Detector()
    node.get_logger().info("start spining detectron_node...")
    
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()