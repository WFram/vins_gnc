from mmseg.apis import inference_model, init_model, show_result_pyplot

import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header
import cv2
import re
import numpy as np
import time


class SemanticSegmentation(object):
    def __init__(self):
        self.cv_bridge = CvBridge()

        self.input_topic = rospy.get_param('~segmentation_input_topic')
        self.output_topic = rospy.get_param('~segmentation_output_topic')
        rospy.loginfo("NN: Segment images from " + self.input_topic + " into " + self.output_topic)

        self.config_file = rospy.get_param('~config_file')
        self.checkpoint_file = rospy.get_param('~checkpoint_file')

        self.model = init_model(self.config_file, self.checkpoint_file, device='cuda:0')

        self.input_msg_queue = []

        self.max_inference_time = 0.080  # Other options: 0.060, 0.125

        self.frame_id = None
        self.stamp = None
        self.incoming_image_id = 0
        self.frames_to_skip = 0

        self.publisher = rospy.Publisher(self.output_topic, Image, queue_size=1)
        self.subscriber = rospy.Subscriber(self.input_topic, Image, self.image_cbk, queue_size=1)
        rospy.loginfo("NN: Start processing")

    def publish(self, semantic_image, frame_id, stamp_now):
        msg = self.cv_bridge.cv2_to_imgmsg(semantic_image, encoding='mono8')
        msg.header.frame_id = frame_id
        msg.header.stamp = stamp_now
        self.publisher.publish(msg)

    def get_predictions(self, data_sample):
        pixel_data = data_sample.pred_sem_seg
        predictions = pixel_data.values()[0].cpu().numpy()
        return predictions[0]

    def save_semantic_image(self, segmentation_result):
        semantic_image = self.get_predictions(segmentation_result)
        semantic_image = semantic_image.astype(np.uint8)
        self.publish(semantic_image, self.frame_id, self.stamp)

    def segment(self):
        msg = self.input_msg_queue[-1]
        self.input_msg_queue.clear()

        #  TODO: Make a condition here
        tmp_raw_image = self.cv_bridge.imgmsg_to_cv2(msg)
        raw_image = tmp_raw_image.copy()

        # raw_image = np.empty((tmp_raw_image.shape[0], tmp_raw_image.shape[1], 3))

        # for ch in range(3):
        #     raw_image[:, :, ch] = tmp_raw_image[:, :]

        time_start = time.time()
        segmentation_result = inference_model(self.model, raw_image)
        time_end = time.time()
        time_duration = time_end - time_start
        rospy.loginfo("Inference time: " + str(time_duration))

        # TODO: think of extra timestamp checking
        if time_duration > self.max_inference_time:
            return

        self.frame_id = msg.header.frame_id
        self.stamp = msg.header.stamp

        self.save_semantic_image(segmentation_result)

    def image_cbk(self, msg):

        self.input_msg_queue.append(msg)

        if self.incoming_image_id % (self.frames_to_skip + 1) == 0:
            self.incoming_image_id += 1
            self.segment()
        else:
            self.incoming_image_id += + 1
            self.input_msg_queue.clear()


if __name__ == '__main__':
    rospy.init_node("semantic_segmentation_node", anonymous=True)
    seg = SemanticSegmentation()
    rospy.spin()
