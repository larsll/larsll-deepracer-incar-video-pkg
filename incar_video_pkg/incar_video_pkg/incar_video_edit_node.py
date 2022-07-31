#!/usr/bin/env python3

import logging
import time
from threading import Thread, Event
import queue
import cv2
import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from cv_bridge import CvBridge
from sensor_msgs.msg import Image as ROSImg

from incar_video_pkg.constants import (PUBLISH_SENSOR_TOPIC, PUBLISH_VIDEO_TOPIC,
                                  Mp4Parameter, MAX_FRAMES_IN_QUEUE, QUEUE_WAIT_TIME, VIDEO_STATE_SRV)

from incar_video_pkg.image_editing import ImageEditing
from incar_video_pkg.logger import Logger

from deepracer_interfaces_pkg.msg import EvoSensorMsg
from deepracer_interfaces_pkg.srv import VideoStateSrv

from incar_video_pkg import utils
from incar_video_pkg.utils import DoubleBuffer


class InCarVideoEditNode(Node):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """
    _edit_queue = list()

    def __init__(self):
        super().__init__('incar_video_edit_node')

        self.recording_active = Event()

        self.declare_parameter('racecar_name', 'DeepRacer', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('publish_stream', False, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('save_to_mp4', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('output_file_name', 'deepracer-{}.mp4', ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('fps', Mp4Parameter.FPS.value, ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))

        self._racecar_name = self.get_parameter('racecar_name').value
        self._publish_to_topic = self.get_parameter('publish_stream').value
        self._save_to_mp4 = self.get_parameter('save_to_mp4').value
        self._output_file_name = self.get_parameter('output_file_name').value
        self._fps = self.get_parameter('fps').value

        if "{}" in self._output_file_name:
            self._output_file_name = self._output_file_name.format(time.strftime("%Y%m%d-%H%M%S"))

        # init cv bridge
        self.bridge = CvBridge()

        # Two job types are required because in the F1 editing we have static variables
        # to compute the gap and ranking. With Mp4 stacking frames, these values would be already updated by KVS.
        # If same class is used then during the finish phase you see all the racers information at once
        # and not updated real time when racers finish the lap.
        # %TODO seperate out the kvs and Mp4 functionality
        #
        self.job_type_image_edit_mp4 = ImageEditing(self._racecar_name)

        # All Mp4 related initialization
        self._edit_queue = queue.Queue()
        self._edit_queue_pushed = 0
        self._last_image_seen = 0


    def __enter__(self):
        self._camera_input_buffer = DoubleBuffer(clear_data_on_get=True)
        self.camera_sub_cbg = ReentrantCallbackGroup()
        self.camera_sub = self.create_subscription(EvoSensorMsg,
                                 PUBLISH_SENSOR_TOPIC,
                                 self._receive_camera_frame_callback,
                                 5,
                                 callback_group=self.camera_sub_cbg)

        # Call ROS service to enable the Video Stream
        self.cli = self.create_client(VideoStateSrv, VIDEO_STATE_SRV)
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Camera service not available, waiting...')

        self.get_logger().info("Camera service available, enabling video stream.")
        req = VideoStateSrv.Request()
        req.activate_video = 1
        _ = self.cli.call_async(req)
        self.recording_active.set()

        # self.subscribe_to_save_mp4()
        self.edit_frame_thread = Thread(target=self._edit_frame_thread)

        # Publisher to broadcast the edited video stream.
        if self._publish_to_topic:
            self.camera_cbg = ReentrantCallbackGroup()
            self.camera_pub = self.create_publisher(ROSImg,
                                    PUBLISH_VIDEO_TOPIC,
                                    250,
                                    callback_group=self.camera_cbg)

        # Saving to MP4
        if self._save_to_mp4:
            self.cv2_video_writer = cv2.VideoWriter(self._output_file_name, Mp4Parameter.FOURCC.value, self._fps, Mp4Parameter.FRAME_SIZE.value)

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping.')
        self.recording_active.clear()
        self.edit_frame_thread.join()
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

    def _receive_camera_frame_callback(self, frame):
        """ Callback for the main input. Once a new image is received, all the required
        service calls are made to get the video metric information. Then for Mp4 its put into the queue
        but for the KVS its put into a double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if rclpy.ok():
            self._edit_queue_pushed += 1
            self.get_logger().debug("Pushed {} frames to Frame Buffer.".format(self._edit_queue_pushed ))

            self._camera_input_buffer.put(frame)

            # On first receive start thread and timer
            if not self.edit_frame_thread.is_alive() and self.recording_active.is_set():
                self.buffer_timer = self.create_timer(1.0/(2 * self._fps), self._buffer_timer_callback)
                self.edit_frame_thread.start()

    def _buffer_timer_callback(self):
        """ Callback for the buffer timer. It reads the current picture from the _camera_input_buffer,
            and places it into the edit frame queue. It will report on errors if frames are dropped.
            It will also terminate video editing if no frames are received.
        """
        try:
            frame = self._camera_input_buffer.get(block=True, timeout=QUEUE_WAIT_TIME)

            if self._edit_queue.qsize() == MAX_FRAMES_IN_QUEUE:
                self.get_logger().info("Dropping Mp4 frame from the queue")
                self._edit_queue.get()
                                  
            new_image_seen = int(frame.images[0].header.frame_id)

            if (new_image_seen - self._last_image_seen > 1):
                self.get_logger().warn(f"Image gap: { new_image_seen } of {new_image_seen - self._last_image_seen}")

            self._last_image_seen = new_image_seen

            # Append to the MP4 queue
            self._edit_queue.put(frame)
            self._edit_queue_pushed += 1

            self.get_logger().debug("Pushed {} frames to Edit Queue.".format(self._edit_queue_pushed))

        except DoubleBuffer.Empty:
            self.get_logger().info("Input buffer is empty for {} seconds. Stopping.".format(QUEUE_WAIT_TIME))
            self.destroy_timer(self.buffer_timer)            
            self.recording_active.clear()
            return

    def _edit_frame_thread(self):
        """ Consumes the frame queued by the _buffer_timer_callback and edits the image
        The edited image is published or written to MP4 file.
        """

        edited_frame_count = 0
        bridge = CvBridge()

        while rclpy.ok() and self.recording_active.is_set():
            frame_data = None
            try:
                # Pop from the queue and edit the image
                frame_data = self._edit_queue.get(block=False)

                if frame_data:
                    edited_frame = self._edit_main_camera_image(frame_data)
                    
                    if self._save_to_mp4:
                        cv_image = bridge.imgmsg_to_cv2(edited_frame, "bgr8")
                        self.cv2_video_writer.write(cv_image)

                    if self._publish_to_topic:
                        self.camera_pub.publish(edited_frame)

                    edited_frame_count += 1
                    self.get_logger().info("Published {} frames. {} frames in queue.".format(edited_frame_count, self._edit_queue.qsize()))
                
            except queue.Empty:
                self.get_logger().debug("Frame buffer is empty")


        if self._save_to_mp4:
            self.cv2_video_writer.release()
            self.get_logger().info("Video written to {}.".format(self._output_file_name))

        if self._publish_to_topic:
            self.destroy_publisher(self.camera_pub)

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
    except:
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
