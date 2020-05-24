import rospy
from styx_msgs.msg import TrafficLight
import rospkg
import os,sys
import tensorflow as tf
import numpy as np
import time
import os
import cv2

IS_DEBUG = True

class TLClassifier(object):

    def __init__(self, is_real_world=False): # Check if we are getting images from simulator or real world
        self.__model_loaded = False
        self.session = None
        self.tf_graph = None
        self.prediction = None
        self.model_path = '../../../models/'

        self.load_model(is_real_world)

    def load_model(self, is_real_world):

        if is_real_world:
            self.model_path += 'ssd_mobilenet_v1_coco_real_dataset.pb'
        else:
            self.model_path += 'ssd_mobilenet_v1_coco_sim_dataset.pb'

        rospy.loginfo('Loading model from' + self.model_path)

        # Loading the graph
        self.tf_graph = load_graph(self.model_path)
        self.config = tf.ConfigProto(log_device_placement=False)

        # GPU video memory usage setup
        self.config.gpu_options.per_process_gpu_memory_fraction = 0.8
        #Setup timeout for any inactive option
        self.config.operation_timeout_in_ms = 50000

        with self.tf_graph.as_default():
            self.tf_session = tf.Session(graph=self.tf_graph, config=self.config)

        self.__model_loaded = True
        rospy.loginfo("Successfully loaded model")

    def get_classification(self, image, min_score=0.5):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if not self.__model_loaded:
            return TrafficLight.UNKNOWN

        tfl_type = ["RED", "YELLOW", "GREEN"]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)

        boxes, scores, classes, num = self.do_computation(image_np)

        # Removing unecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32)

        for j, box in enumerate(boxes):
            width = (box[3] - box[1]) * image.shape[1]
            height = (box[2] - box[0]) * image.shape[0]

            # skip if traffic light image is less than 50px - too far or nonexistent
            if height < 50:
                if IS_DEBUG:
                    rospy.loginfo("No traffic light or too far away")
                return TrafficLight.UNKNOWN
            else:
                # throwing away results below threshold
                final_scores, final_classes = get_final_results(min_score, scores, classes)

                if len(final_classes) == 0:
                    if IS_DEBUG:
                        rospy.loginfo("No traffic light is detected")
                    return TrafficLight.UNKNOWN

                # TrafficLight messages have red = 0, yellow = 1, green = 2.
                # The model is trained to identify class red = 1, yellow = 2, green = 3.
                # so, subtracting 1 to match with TrafficLight message spec.
                if IS_DEBUG:
                    rospy.loginfo("Predicted TFL color is " + tfl_type[final_classes[0] - 1] + " and score is " + str(final_scores[0]))
                return final_classes[0] - 1

    def do_computation(self, image_np):
        # Declaring our placeholders
        image_tensor = self.tf_graph.get_tensor_by_name('prefix/image_tensor:0')

        # Each box represents a part of the image where a particular object was detected.
        detection_scores = self.tf_graph.get_tensor_by_name('prefix/detection_scores:0')

        # Number of predictions found in the image
        num_detections = self.tf_graph.get_tensor_by_name('prefix/num_detections:0')

        # Classification of the object (integer id)
        detection_classes = self.tf_graph.get_tensor_by_name('prefix/detection_classes:0')

        detection_boxes = self.tf_graph.get_tensor_by_name('prefix/detection_boxes:0')

        # Get the scores, classes and number of detections
        # re-use the session: it is more than 3x faster than creating a new one for every image
        (boxes, scores, classes, num) = self.tf_session.run(
            [detection_boxes, detection_scores, detection_classes, num_detections],
            feed_dict={image_tensor: image_np})

        return boxes, scores, classes, num


def load_graph (graph_file):
    graph = tf.Graph()
    with graph.as_default():
        od_graph_def = tf.GraphDef()
        with tf.gfile.GFile(graph_file, 'rb') as fid:
            serialized_graph = fid.read()
            od_graph_def.ParseFromString(serialized_graph)
            tf.import_graph_def(od_graph_def, name='prefix')
    return graph


def get_final_results(min_score, scores, classes):
    indexes = []
    for i in range(len(classes)):
        if scores[i] >= min_score:
            indexes.append(i)


    final_scores = scores[indexes, ...]
    final_classes = classes[indexes, ...]
    return final_scores, final_classes
