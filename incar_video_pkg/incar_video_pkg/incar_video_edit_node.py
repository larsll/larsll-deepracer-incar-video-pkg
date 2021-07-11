#!/usr/bin/env python3
##############################################################
#                                                            #
#   Copyright 2019 Amazon.com, Inc. or its affiliates.       #
#   All Rights Reserved.                                     #
#                                                            #
##############################################################
import sys
import time
import logging
from threading import Thread
import queue
import cv2
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.exceptions import (ROSInterruptException)
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Empty
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as ROSImg
from std_msgs.msg import String

from incar_video_pkg.constants import (RaceType, CameraTypeParams, PUBLISH_SENSOR_TOPIC, PUBLISH_VIDEO_TOPIC,
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
    _mp4_queue = list()

    def __init__(self, racecar_name="DeepRacer", is_publish_to_live_stream=True):
        super().__init__('incar_video_edit_node')
        #
        # We have no guarantees as to when gazebo will load the model, therefore we need
        # to wait until the model is loaded and markov packages has spawned all the models
        #
        ## rclpy.wait_for_service('/robomaker_markov_package_ready')

        self._agents_metrics.append(DoubleBuffer(clear_data_on_get=False))

        self.declare_parameter('VIDEO_JOB_TYPE', "RACING")
        self.declare_parameter('LEADERBOARD_TYPE', "LEAGUE")
        self.declare_parameter('LEADERBOARD_NAME', "TEST")
        self.declare_parameter('NUMBER_OF_TRIALS', 1)

        self.racecar_name = racecar_name
        self.racecar_index = 0
        self._is_publish_to_live_stream = is_publish_to_live_stream
        self.is_training = False

        # init cv bridge
        self.bridge = CvBridge()

        # This determines what kind of image editing should be done based on the race type
        self.race_type = RaceType.LIVE.value
        self.top_camera_mp4_pub = False
        #
        # Two job types are required because in the F1 editing we have static variables
        # to compute the gap and ranking. With Mp4 stacking frames, these values would be already updated by KVS.
        # If same class is used then during the finish phase you see all the racers information at once
        # and not updated real time when racers finish the lap.
        # %TODO seperate out the kvs and Mp4 functionality
        #
        self.job_type_image_edit_mp4 = ImageEditing(self.racecar_name, None, self.race_type)

        # All Mp4 related initialization
        self._mp4_queue.append(queue.Queue())
        self._mp4_queue_pushed = 0

        # Initialize save mp4 ROS service for the markov package to signal when to
        # start and stop collecting video frames
        camera_info = utils.get_cameratype_params(self.racecar_name, self.racecar_name)
        self.save_to_mp4_obj = SaveToMp4(camera_infos=[camera_info[CameraTypeParams.CAMERA_FORWARD_PARAMS]],
                                         fourcc=Mp4Parameter.FOURCC.value,
                                         fps=Mp4Parameter.FPS.value,
                                         frame_size=Mp4Parameter.FRAME_SIZE.value)

        # Publisher to broadcast the notification messages.
        self.camera_cbg = ReentrantCallbackGroup()
        self.camera_pub = self.create_publisher(ROSImg,
                                  PUBLISH_VIDEO_TOPIC,
                                  10,
                                  callback_group=self.camera_cbg)

    def __enter__(self):
        self._main_camera_frame_buffer = DoubleBuffer(clear_data_on_get=True)
        self.camera_sub_cbg = ReentrantCallbackGroup()
        self.camera_sub = self.create_subscription(EvoSensorMsg,
                                 PUBLISH_SENSOR_TOPIC,
                                 self._producer_frame_callback,
                                 1,
                                 callback_group=self.camera_sub_cbg)

        self.subscribe_to_save_mp4()
        self.consumer_thread = Thread(target=self._consumer_mp4_frame_thread)
        self.consumer_thread.start()

        self.throttle_thread = Thread(target=self._throttle_thread)
        self.throttle_thread.start()

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.frame_rate.destroy()
        self.unsubscribe_to_save_mp4()
        self.get_logger().info('Exiting.')

    def subscribe_to_save_mp4(self):
        """ Ros service handler function used to subscribe to the Image topic.
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        self.is_save_mp4_enabled = True
        self.save_to_mp4_obj.subscribe_to_save_mp4(self)
        return []

    def unsubscribe_to_save_mp4(self):
        """ Ros service handler function used to unsubscribe from the Image topic.
        This will take care of cleaning and releasing the cv2 VideoWriter
        Arguments:
            req (req): Dummy req else the ros service throws exception
        Return:
            [] - Empty list else ros service throws exception
        """
        self.is_save_mp4_enabled = True
        # This is required because when unsubscribe call is made the frames in the queue will continue editing,
        # but at this time the 45degree camera will continue to be subscribed and saved to mp4 which we do not want.
        camera_topics_stop_immediately, camera_topics_stop_post_empty_queue = list(), list()
        if not self.top_camera_mp4_pub:
            camera_topics_stop_immediately = [CameraTypeParams.CAMERA_45DEGREE_PARAMS.value,
                                              CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value]
            camera_topics_stop_post_empty_queue = [CameraTypeParams.CAMERA_PIP_PARAMS.value,
                                                    CameraTypeParams.CAMERA_FORWARD_PARAMS.value]
        else:
            camera_topics_stop_immediately = [CameraTypeParams.CAMERA_45DEGREE_PARAMS.value]
            camera_topics_stop_post_empty_queue = [CameraTypeParams.CAMERA_TOPVIEW_PARAMS.value,
                                                   CameraTypeParams.CAMERA_PIP_PARAMS.value,
                                                   CameraTypeParams.CAMERA_FORWARD_PARAMS.value]

        self.save_to_mp4_obj.unsubscribe_to_save_mp4(camera_topics_stop_immediately)
        LOG.info("Waiting to flush the Mp4 queue for racecar_{}...".format(self.racecar_index))
        while not self._mp4_queue[self.racecar_index].empty():
            time.sleep(1)
        LOG.info("Done flushing the Mp4 queue for racecar_{}...".format(self.racecar_index))
        self.save_to_mp4_obj.unsubscribe_to_save_mp4(camera_topics_stop_post_empty_queue)

        
    def _edit_main_camera_images(self, frame_data, edited_frame_result):
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

        edited_main_frame = self.bridge.cv2_to_imgmsg(major_cv_image, "bgr8")
        edited_frame_result[FrameTypes.MAIN_CAMERA_FRAME.value] = edited_main_frame

    def _edit_camera_images(self, frame_data, is_mp4):
        """ Edit camera image by calling respective job type

        Arguments:
            frame_data (dict): Dictionary of frame, agent_metric_info, training_phase
            is_mp4 (bool): Is this edit camera image for kvs or mp4

        Returns:
            Image: Edited image
        """
        # convert ros image message to cv image
        try:
            edited_frame_result = dict()
            self._edit_main_camera_images(frame_data, edited_frame_result)
            return edited_frame_result
        except CvBridgeError as ex:
            LOG.info("cv2 to ROS image message error: {}".format(ex))

    def _producer_frame_callback(self, frame):
        """ Callback for the main input. Once a new image is received, all the required
        service calls are made to get the video metric information. Then for Mp4 its put into the queue
        but for the KVS its put into a double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if rclpy.ok():
            self._main_camera_frame_buffer.put(frame)


    def _throttle_thread(self):

        self.frame_rate = self.create_rate(Mp4Parameter.FPS.value)
        mp4_queue_pushed = 0

        while rclpy.ok():
            try:
                frame = self._main_camera_frame_buffer.get(block=True, timeout=QUEUE_WAIT_TIME)

                if self._mp4_queue[self.racecar_index].qsize() == MAX_FRAMES_IN_QUEUE:
                    LOG.info("Dropping Mp4 frame from the queue")
                    self._mp4_queue[self.racecar_index].get()
                
                # Append to the MP4 queue
                self._mp4_queue[self.racecar_index].put(frame)
                mp4_queue_pushed += 1

                LOG.info("Pushed {} frames to MP4 Queue.".format(mp4_queue_pushed))

                if not self.frame_rate._is_destroyed: 
                    self.frame_rate.sleep()
                else:
                    LOG.info("Stopped throttle thread.")
                    return

            except queue.Empty:
                LOG.info("Input buffer is empty. Stopping")
                return

    def _consumer_mp4_frame_thread(self):
        """ Consumes the frame produced by the _producer_frame_thread and edits the image
        The edited image is put into another queue for publishing to MP4 topic
        """

        mp4_queue_published = 0

        while rclpy.ok():
            frame_data = None
            try:
                # Pop from the queue and edit the image
                frame_data = self._mp4_queue[self.racecar_index].get(block=True, timeout=QUEUE_WAIT_TIME)
            except queue.Empty:
                LOG.info("AgentsVideoEditor._mp4_queue['{}'] is empty. Stopping.".format(self.racecar_index))
                self.unsubscribe_to_save_mp4()
                return

            if frame_data:
                edited_frames = self._edit_camera_images(frame_data, is_mp4=True)
                
                self.camera_pub.publish(edited_frames[FrameTypes.MAIN_CAMERA_FRAME.value])
                mp4_queue_published += 1
                LOG.info("Published {} frames.".format(mp4_queue_published))

def main(args=None):
    rclpy.init(args=args)
    with InCarVideoEditNode() as incar_video_edit_node:
        executor = MultiThreadedExecutor()
        rclpy.spin(incar_video_edit_node, executor)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    incar_video_edit_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
