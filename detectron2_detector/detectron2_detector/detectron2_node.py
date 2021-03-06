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

from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import Image, PointCloud2 
from nav2_dynamic_msgs.msg import Obstacle, ObstacleArray
from geometry_msgs.msg import Pose, Point
from detectron2_detector.utils import NMS_3D

class Detectron2Detector(Node):
    '''use Detectron2 to detect object masks from 2D image and estimate 3D position with Pointcloud2 data
    '''
    def __init__(self):
        super().__init__('detectron_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('global_frame', "camera_link"),
                ('detectron_config_file', "COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"),
                ('detectron_score_thresh', 0.8),
                ('pointcloud2_topic', "/realsense/camera/pointcloud"),
                ('pc_downsample_factor', 16),
                ('min_mask', 20),
                ('categories', [0]),
                ('z_filter', [-2., 2.]),
                ('nms_filter', 0.3)
            ])
        self.global_frame = self.get_parameter("global_frame")._value
        self.pc_downsample_factor = int(self.get_parameter("pc_downsample_factor")._value)
        self.min_mask = self.get_parameter("min_mask")._value
        self.categories = self.get_parameter("categories")._value
        self.z_filter = self.get_parameter("z_filter")._value
        self.nms_filter = self.get_parameter("nms_filter")._value

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
        self.detect_obj_pub = self.create_publisher(ObstacleArray, 'detection', 2)
        self.detect_img_pub = self.create_publisher(Image, 'image', 2)

        self.count = -1

        # setup tf related
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        # check if there is subscirbers
        if self.detect_obj_pub.get_subscription_count() == 0 and self.detect_img_pub.get_subscription_count() == 0:
            return

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
        scores = outputs["instances"].scores.cpu().numpy().astype(np.float)

        # estimate 3D position with simple averaging of obstacle's points
        obstacle_array = ObstacleArray()
        obstacle_array.header = msg.header
        detections = []
        for i in range(num_classes):
            # if user does not specify any interested category, keep all; else select those interested objects
            if (len(self.categories) == 0) or (outputs["instances"].pred_classes[i] in self.categories):
                idx = np.where(masks[i])[0]
                idx = self.outlier_filter(x[idx], z[idx], idx)
                if idx.shape[0] < self.min_mask:
                    continue
                obstacle_msg = Obstacle()
                # pointcloud2 data has a different coordinate, swap y and z
                # use (max+min)/2 can avoid the affect of unbalance of points density instead of average
                x_max = x[idx].max()
                x_min = x[idx].min()
                y_max = y[idx].max()
                y_min = y[idx].min()
                z_max = z[idx].max()
                z_min = z[idx].min()
                obstacle_msg.score = scores[i]
                obstacle_msg.position.x = np.float((x_max + x_min) / 2)
                obstacle_msg.position.y = np.float((y_max + y_min) / 2)
                obstacle_msg.position.z = np.float((z_max + z_min) / 2)
                obstacle_msg.size.x = np.float(x_max - x_min)
                obstacle_msg.size.y = np.float(y_max - y_min)
                obstacle_msg.size.z = np.float(z_max - z_min)
                detections.append(obstacle_msg)

        # transform to global frame
        if self.global_frame is not None:
            try:
                trans = self.tf_buffer.lookup_transform(self.global_frame, msg.header.frame_id, rclpy.time.Time())
            except LookupException:
                self.get_logger().info('fail to get tf from {} to {}'.format(msg.header.frame_id, self.global_frame))
                return
            else:
                # TODO: use tf2_geometry_msgs when it can be imported to python in ros2
                r = R.from_quat([trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w])
                msg.header.frame_id = self.global_frame
                for i in range(len(detections)):
                    p = [detections[i].position.x, detections[i].position.y, detections[i].position.z]
                    p = r.apply(p)
                    detections[i].position.x = p[0] + trans.transform.translation.x 
                    detections[i].position.y = p[1] + trans.transform.translation.y
                    detections[i].position.z = p[2] + trans.transform.translation.z
                    s = [detections[i].size.x, detections[i].size.y, detections[i].size.z]
                    s = r.apply(s)
                    detections[i].size.x = s[0]
                    detections[i].size.y = s[1]
                    detections[i].size.z = s[2]

        # filter detections
        # non-max suppression
        detections = NMS_3D(detections, self.nms_filter)
        # filter z axis
        new_detections = []
        for det in detections:
            if det.position.z > self.z_filter[0] and det.position.z < self.z_filter[1]:
                new_detections.append(det)
        detections = new_detections

        # publish detection result 
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