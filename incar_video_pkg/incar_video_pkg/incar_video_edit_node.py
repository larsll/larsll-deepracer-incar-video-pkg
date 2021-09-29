#!/usr/bin/env python3
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################

import logging
from threading import Thread, Event
import queue
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImg

from incar_video_pkg.constants import (RaceType, PUBLISH_SENSOR_TOPIC, PUBLISH_VIDEO_TOPIC,
                                  RaceCarColorToRGB, Mp4Parameter, FrameQueueData, MAX_FRAMES_IN_QUEUE,
                                  KVS_PUBLISH_PERIOD, QUEUE_WAIT_TIME, FrameTypes)

from incar_video_pkg.image_editing import ImageEditing
from incar_video_pkg.logger import Logger

from deepracer_interfaces_pkg.msg import EvoSensorMsg

from incar_video_pkg import utils
from incar_video_pkg.utils import DoubleBuffer, force_list, log_and_exit
from incar_video_pkg.save_to_mp4 import SaveToMp4

LOG = Logger(__name__, logging.INFO).get_logger()


class InCarVideoEditNode(Node):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """
    _agents_metrics = list()
    _frame_queue = list()

    def __init__(self):
        super().__init__('incar_video_edit_node')

        self.stop_node = Event()
        #
        # We have no guarantees as to when gazebo will load the model, therefore we need
        # to wait until the model is loaded and markov packages has spawned all the models
        #
        ## rclpy.wait_for_service('/robomaker_markov_package_ready')

        self._agents_metrics.append(DoubleBuffer(clear_data_on_get=False))

        self.declare_parameter('racecar_name', 'DeepRacer', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('publish_stream', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('save_to_mp4', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        self._racecar_name = self.get_parameter('racecar_name').value
        self._publish_to_topic = self.get_parameter('publish_stream').value
        self._save_to_mp4 = self.get_parameter('save_to_mp4').value

        # init cv bridge
        self.bridge = CvBridge()

        # Two job types are required because in the F1 editing we have static variables
        # to compute the gap and ranking. With Mp4 stacking frames, these values would be already updated by KVS.
        # If same class is used then during the finish phase you see all the racers information at once
        # and not updated real time when racers finish the lap.
        # %TODO seperate out the kvs and Mp4 functionality
        #
        self.job_type_image_edit_mp4 = ImageEditing(self._racecar_name, None, None)

        # All Mp4 related initialization
        self._frame_queue = queue.Queue()
        self._mp4_queue_pushed = 0
        self._edit_queue_pushed = 0


    def __enter__(self):
        self._main_camera_frame_buffer = DoubleBuffer(clear_data_on_get=True)
        self.camera_sub_cbg = ReentrantCallbackGroup()
        self.camera_sub = self.create_subscription(EvoSensorMsg,
                                 PUBLISH_SENSOR_TOPIC,
                                 self._producer_frame_callback,
                                 5,
                                 callback_group=self.camera_sub_cbg)

        # self.subscribe_to_save_mp4()
        self.consumer_thread = Thread(target=self._consumer_mp4_frame_thread)
        self.consumer_thread.start()

        # Publisher to broadcast the edited video stream.
        if self._publish_to_topic:
            self.camera_cbg = ReentrantCallbackGroup()
            self.camera_pub = self.create_publisher(ROSImg,
                                    PUBLISH_VIDEO_TOPIC,
                                    250,
                                    callback_group=self.camera_cbg)

        # Saving to MP4
        if self._save_to_mp4:
            self.cv2_video_writer = cv2.VideoWriter('output/video.mp4', Mp4Parameter.FOURCC.value, Mp4Parameter.FPS.value, Mp4Parameter.FRAME_SIZE.value)

        self.throttle_timer = self.create_timer(1.0/Mp4Parameter.FPS.value, self._throttle_timer_callback)

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping.')
        self.stop_node.set()
        self.consumer_thread.join()
        self.get_logger().info('Done.')
        
    def _edit_main_camera_image(self, frame_data):
        """ Thread to edit main camera frames

        Args:
            frame_data (EvoSensorMsg): Dictionary of frame, agent_metric_info, training_phase
            edited_frame_result (dict): A mutable variable holding the dict result of edited frame
        """
        main_frame = frame_data.images[0]
        major_cv_image = self.bridge.imgmsg_to_cv2(main_frame, "bgr8")
        major_cv_image = cv2.cvtColor(major_cv_image, cv2.COLOR_RGB2RGBA)
        # Edit the image based on the racecar type and job type

        major_cv_image = self.job_type_image_edit_mp4.edit_image(major_cv_image, frame_data.imu_data)

        return self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")

    def _producer_frame_callback(self, frame):
        """ Callback for the main input. Once a new image is received, all the required
        service calls are made to get the video metric information. Then for Mp4 its put into the queue
        but for the KVS its put into a double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if rclpy.ok():
            self._edit_queue_pushed += 1
            LOG.debug("Pushed {} frames to Frame Buffer.".format(self._edit_queue_pushed ))

            self._main_camera_frame_buffer.put(frame)


    def _throttle_timer_callback(self):

        try:
            frame = self._main_camera_frame_buffer.get(block=True, timeout=QUEUE_WAIT_TIME)

            if self._frame_queue.qsize() == MAX_FRAMES_IN_QUEUE:
                LOG.info("Dropping Mp4 frame from the queue")
                self._frame_queue.get()
            
            # Append to the MP4 queue
            self._frame_queue.put(frame)
            self._mp4_queue_pushed += 1

            LOG.debug("Pushed {} frames to Edit Queue.".format(self._mp4_queue_pushed))

        except queue.Empty:
            LOG.info("Input buffer is empty. Stopping")
            return

    def _consumer_mp4_frame_thread(self):
        """ Consumes the frame produced by the _producer_frame_thread and edits the image
        The edited image is put into another queue for publishing to MP4 topic
        """

        mp4_queue_published = 0
        bridge = CvBridge()

        while rclpy.ok() and not self.stop_node.is_set():
            frame_data = None
            try:
                # Pop from the queue and edit the image
                frame_data = self._frame_queue.get(block=True, timeout=QUEUE_WAIT_TIME)

                if frame_data:
                    edited_frame = self._edit_main_camera_image(frame_data)
                    
                    if self._save_to_mp4:
                        cv_image = bridge.imgmsg_to_cv2(edited_frame, "bgr8")
                        self.cv2_video_writer.write(cv_image)

                    if self._publish_to_topic:
                        self.camera_pub.publish(edited_frame)

                    mp4_queue_published += 1
                    LOG.info("Published {} frame to MP4 queue. {} frames in queue.".format(mp4_queue_published, self._frame_queue.qsize()))
                
            except queue.Empty:
                LOG.info("Frame buffer is empty. Stopping.")
                self.stop_node.set()

        if self._save_to_mp4:
            self.cv2_video_writer.release()

        if self._publish_to_topic:
            self.destroy_publisher(self.camera_pub)

        self.destroy_timer(self.throttle_timer)
        self.destroy_subscription(self.camera_sub)

def main(args=None):

    try:
        rclpy.init(args=args)
        with InCarVideoEditNode() as incar_video_edit_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(incar_video_edit_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        incar_video_edit_node.destroy_node()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == "__main__":
    main()
