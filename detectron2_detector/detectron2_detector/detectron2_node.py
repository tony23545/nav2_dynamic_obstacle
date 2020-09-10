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
import numpy as np
import matplotlib.pyplot as plt
from nav2_dynamic_msgs.msg import ObjectCircle, ObjectCircleArray

class Detectron2Detector(Node):
    def __init__(self):
        super().__init__('detectron_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('detectron_config_file', None),
                ('pointcloud2_topic', None),
            ])

        # setup detectron model
        self.cfg = get_cfg()
        config_file = self.get_parameter("detectron_config_file")._value
        self.cfg.merge_from_file(model_zoo.get_config_file(config_file))
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5 
        self.cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(config_file)
        self.predictor = DefaultPredictor(self.cfg)

        # subscribe to sensor 
        self.subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("pointcloud2_topic")._value,
            self.callback,
            1)

        # setup publisher
        self.detect_obj_pub = self.create_publisher(ObjectCircleArray, 'detectron2_object_circle', 10)
        self.detect_img_pub = self.create_publisher(Image, 'detectron2_image', 10)

        self.count = -1

    def outlier_filter(self, x, z, idx):
        # simple outlier filter, drop points out of 3sigma
        x_mean = np.mean(x)
        x_var = np.var(x)
        z_mean = np.mean(z)
        z_var = np.var(z)
        gaussian_kernel = np.exp(-0.5 * (np.power(x-x_mean, 2) / x_var + np.power(z-z_mean, 2) / z_var)) / (2 * np.pi * np.sqrt(x_var * z_var))
        return idx[gaussian_kernel > 0.5]


    def callback(self, msg):
        self.get_logger().info("processing one frame...")

        height = msg.height
        width = msg.width
        points = np.array(msg.data, dtype = 'uint8')
        # rgb image
        rgb_offset = msg.fields[3].offset
        point_step = msg.point_step
        r = points[rgb_offset::point_step]
        g = points[(rgb_offset+1)::point_step]
        b = points[(rgb_offset+2)::point_step]
        img = np.concatenate([r[:, None], g[:, None], b[:, None]], axis = -1)
        img = img.reshape((height, width, 3))
        # point cloud
        points = points.view('<f4')
        down_sample_scale = 16
        x = points[::int(down_sample_scale  * point_step / 4)]
        y = points[1::int(down_sample_scale * point_step / 4)]
        z = points[2::int(down_sample_scale * point_step / 4)]

        # call detectron model
        outputs = self.predictor(img)

        # map to point cloud data
        color = np.zeros_like(x, dtype = 'uint8')
        num_classes = outputs['instances'].pred_classes.shape[0]
        masks = outputs["instances"].pred_masks.cpu().numpy().astype('uint8').reshape((num_classes, -1))[:, ::down_sample_scale]
        head_count = 0

        object_array = ObjectCircleArray()
        object_array.header = msg.header
        detections = []
        for i in range(num_classes):
            if outputs["instances"].pred_classes[i] == 0:
                idx = np.where(masks[i])[0]
                idx = self.outlier_filter(x[idx], z[idx], idx)
                if idx.shape[0] == 0:
                    continue
                ObjectMsg = ObjectCircle()
                ObjectMsg.x = np.float(x[idx].mean())
                ObjectMsg.y = np.float(z[idx].mean())
                ObjectMsg.r = np.linalg.norm(np.concatenate([x[idx, None], z[idx, None]], axis = -1) - np.array([[ObjectMsg.x, ObjectMsg.y]]), axis = -1).max()
                detections.append(ObjectMsg)
                head_count += 1
                #color[idx] += head_count

        # publisher objects
        object_array.objects = detections
        object_array.object_num = head_count
        self.detect_obj_pub.publish(object_array)

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
        self.get_logger().info("len of raw data: " + str(len(out_img_msg.data)))
        self.detect_img_pub.publish(out_img_msg)

def main():
    rclpy.init(args = None)
    node = Detectron2Detector()
    node.get_logger().info("start spining detectron_node...")
    
    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()