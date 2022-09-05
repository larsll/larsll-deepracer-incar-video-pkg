#!/usr/bin/env python3

import logging
import time
from threading import Thread
import queue
import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import Image as ROSImg
from sensor_msgs.msg import CompressedImage as ROSCImg

from deepracer_interfaces_pkg.msg import EvoSensorMsg
from deepracer_interfaces_pkg.srv import VideoStateSrv

from incar_video_pkg.constants import (
    PUBLISH_COMPRESSED_VIDEO_TOPIC, PUBLISH_SENSOR_TOPIC, PUBLISH_VIDEO_TOPIC,
    RECORDING_STATE_SERVICE_NAME, STATUS_TOPIC, Mp4Parameter,
    MAX_FRAMES_IN_QUEUE, QUEUE_WAIT_TIME, VIDEO_STATE_SRV, RecordingState)
from incar_video_pkg.image_editing import ImageEditing
from incar_video_pkg.utils import DoubleBuffer

from incar_video_interfaces_pkg.msg import StatusMsg
from incar_video_interfaces_pkg.srv import RecordStateSrv


class InCarVideoEditNode(Node):
    """ This node is used to produce frames for the AWS kinesis video stream
    and for saving the mp4 and uploading to S3. Both are subscribed to the
    output of the image topic produced by this node.
    """
    _edit_queue = list()

    def __init__(self):
        super().__init__('incar_video_edit_node')

        self._rec_state = RecordingState.Stopped

        self.declare_parameter(
            'racecar_name', 'DeepRacer',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter('publish_stream', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter('save_to_mp4', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(
            'output_file_name', 'deepracer-{}.mp4',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(
            'fps', Mp4Parameter.FPS.value,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter('publish_status', True, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))

        self._racecar_name = self.get_parameter('racecar_name').value
        self._publish_to_topic = self.get_parameter('publish_stream').value
        self._save_to_mp4 = self.get_parameter('save_to_mp4').value
        self._output_file_name = self.get_parameter('output_file_name').value
        self._fps = self.get_parameter('fps').value
        self._publish_status = self.get_parameter('publish_status').value

        # Init cv bridge and Image Edit code
        self._bridge = CvBridge()
        self.job_type_image_edit_mp4 = ImageEditing(self._racecar_name)

        # All Mp4 related initialization
        self._edit_queue = queue.Queue()
        self._edit_queue_pushed = 0
        self._last_image_seen = 0
        self._edited_frame_count = 0

    def __enter__(self):
        self._camera_input_buffer = DoubleBuffer(clear_data_on_get=True)
        self._camera_sub_cbg = ReentrantCallbackGroup()
        self._camera_sub = self.create_subscription(
            EvoSensorMsg, PUBLISH_SENSOR_TOPIC, self.
            _receive_camera_frame_callback, 5,
            callback_group=self._camera_sub_cbg)

        # Call ROS service to enable the Video Stream
        self._camera_state_cli = self.create_client(
            VideoStateSrv, VIDEO_STATE_SRV)
        while not self._camera_state_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Camera service not available, waiting...')

        self.get_logger().info("Camera service available, enabling video"
                               " stream.")
        _ = self._camera_state_cli.call_async(
            VideoStateSrv.Request(activate_video=1))

        # Publisher to broadcast the edited video stream.
        if self._publish_to_topic:
            self._camera_cbg = ReentrantCallbackGroup()
            self._camera_pub = self.create_publisher(
                ROSImg, PUBLISH_VIDEO_TOPIC, 250,
                callback_group=self._camera_cbg)
            self._camera_cpub = self.create_publisher(
                ROSCImg, PUBLISH_COMPRESSED_VIDEO_TOPIC, 250,
                callback_group=self._camera_cbg)

        # Publisher for status messages
        if self._publish_status:
            self._status_cbg = ReentrantCallbackGroup()
            self._status_pub = self.create_publisher(
                StatusMsg, STATUS_TOPIC, 1, callback_group=self._status_cbg)
            self.status_timer = self.create_timer(1,
                                                  self._status_timer_callback)

        # Service to start and stop recording
        self._state_service_cbg = ReentrantCallbackGroup()
        self._state_service = self.create_service(
            RecordStateSrv, RECORDING_STATE_SERVICE_NAME, self.
            _state_service_callback, callback_group=self._state_service_cbg)

        # Preparing the Edit Frame Thread pointer
        self.edit_frame_thread = None

        # Prepare timer
        self.buffer_timer = self.create_timer(1.0/(2 * self._fps),
                                              self._buffer_timer_callback)

        self.get_logger().info('Node started. Ready to start recording.')

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'
                               .format(ExcType.__name__))
        if self._rec_state != RecordingState.Stopped:
            self._rec_state = RecordingState.Stopped
            self.destroy_timer(self.buffer_timer)
            if (self.edit_frame_thread is not None):
                self.edit_frame_thread.join()

        self.get_logger().info('Node cleanup done. Exiting.')

    def _start_recording(self):
        """ Method that is used to start the recording.
        """
        self._edited_frame_count = 0
        self._edit_queue = queue.Queue()
        self._last_image_seen = 0

        self._rec_state = RecordingState.Running
        self.edit_frame_thread = Thread(target=self._edit_frame_thread)
        self.edit_frame_thread.start()

        # Saving to MP4
        if self._save_to_mp4:
            if "{}" in self._output_file_name:
                self._output_file_name_current = self._output_file_name.format(
                    time.strftime("%Y%m%d-%H%M%S"))
            else:
                self._output_file_name_current = self._output_file_name

            self.cv2_video_writer = cv2.VideoWriter(
                self._output_file_name_current, Mp4Parameter.FOURCC.value,
                self._fps, Mp4Parameter.FRAME_SIZE.value)

            self.get_logger().info('Starting recording to {}.'.format(
                self._output_file_name_current))
        else:
            self.get_logger().info('Starting recording.')

    def _stop_recording(self):
        """ Method that is used to stop the recording.
        """
        self.get_logger().info(
            'Stopping the recording with {} frames in the queue.'.format(
                self._edit_queue.qsize()))
        self._rec_state = RecordingState.Stopping

    def _state_service_callback(self, req, res):
        """Callback for the recording state service.
        Args:
            req (RecordingState.Request): Request change to the recording state
            res (RecordingState.Response): Response object with error(int) flag
                                           to indicate if the service call was
                                           successful.

        Returns:
            RecordingState.Response: Response object with error(int) flag to
                                     indicate if the call was successful.
        """
        if self._rec_state == RecordingState.Running and req.state == 0:
            self._stop_recording()
            res.error = 0
            res.desc = "OK"

        elif (self._rec_state == RecordingState.Running) and (req.state == 1):
            res.error = 1
            res.desc = "Recording is already running"

        elif self._rec_state == RecordingState.Stopping and req.state == 1:
            res.error = 1
            res.desc = "Recording is stopping"

        elif self._rec_state == RecordingState.Stopping and req.state == 0:
            res.error = 1
            res.desc = "Recording is stopping"

        elif self._rec_state == RecordingState.Stopped and req.state == 0:
            res.error = 0
            res.desc = "Recording already stopped"

        elif self._rec_state == RecordingState.Stopped and req.state == 1:
            self._start_recording()
            res.error = 0
            res.desc = "OK"

        return res

    def _status_timer_callback(self):
        """Callback that sends out the status message for any node monitoring
        """
        if self._publish_status:
            self._status_pub.publish(
                StatusMsg(
                    state=self._rec_state.value,
                    published=self._edited_frame_count,
                    queue=self._edit_queue.qsize()))

    def _receive_camera_frame_callback(self, frame):
        """ Callback for the main input. Once a new image is received, all the
        required service calls are made to get the video metric information.
        Then for Mp4 its put into the queue but for the KVS its put into a
        double since we only care for the latest image for KVS

        Arguments:
            frame (cv2.ImgMsg): Image/Sensor topic of the camera image frame
        """
        if rclpy.ok():
            self._edit_queue_pushed += 1
            self.get_logger().debug(
                "Pushed {} frames to Frame Buffer.".format(
                    self._edit_queue_pushed))

            self._camera_input_buffer.put(frame)

    def _buffer_timer_callback(self):
        """ Callback for the buffer timer. It reads the current picture from
            the _camera_input_buffer, and places it into the edit frame queue.
            It will report on errors if frames are dropped. It will also
            terminate video editing if no frames are received.
        """
        try:

            if self._rec_state == RecordingState.Running:
                frame = self._camera_input_buffer.get(
                    block=True, timeout=QUEUE_WAIT_TIME)

                if self._edit_queue.qsize() == MAX_FRAMES_IN_QUEUE:
                    self.get_logger().info("Dropping Mp4 frame from the queue")
                    self._edit_queue.get()

                new_image_seen = int(frame.images[0].header.frame_id)

                if (new_image_seen - self._last_image_seen > 1 and
                        self._last_image_seen > 0):
                    self.get_logger().warn(
                        f"Image gap: Frame { new_image_seen }, \
                            missing {new_image_seen - self._last_image_seen}")

                self._last_image_seen = new_image_seen

                # Append to the MP4 queue
                self._edit_queue.put(frame)
                self._edit_queue_pushed += 1

                self.get_logger().debug(
                    "Pushed {} frames to Edit Queue.".format(
                        self._edit_queue_pushed))

        except DoubleBuffer.Empty:
            if self._rec_state == RecordingState.Running:
                self.get_logger().info(
                    "Input buffer is empty for {} seconds. Stopping.".format(
                        QUEUE_WAIT_TIME))
                self.buffer_timer.cancel()
                self._rec_state = RecordingState.Stopping
            return

    def _edit_frame_thread(self):
        """ Consumes the frame queued by the _buffer_timer_callback and edits
        the image. The edited image is published or written to MP4 file.
        """

        while rclpy.ok() and self._rec_state != RecordingState.Stopped:
            frame_data = None
            try:
                # Pop from the queue and edit the image
                frame_data = self._edit_queue.get(block=False)

                if frame_data:
                    main_frame = frame_data.images[0]
                    major_cv_image = self._bridge.compressed_imgmsg_to_cv2(
                        main_frame)
                    major_cv_image = cv2.cvtColor(
                        major_cv_image, cv2.COLOR_RGB2RGBA)
                    edited_frame = self.job_type_image_edit_mp4.edit_image(
                        major_cv_image, frame_data.imu_data)

                    if self._save_to_mp4:
                        self.cv2_video_writer.write(edited_frame)

                    if self._publish_to_topic:
                        self._camera_pub.publish(
                            self._bridge.cv2_to_imgmsg(edited_frame, "bgr8"))
                        c_msg = self._bridge.cv2_to_compressed_imgmsg(
                            edited_frame)
                        c_msg.format = "bgr8; jpeg compressed bgr8"
                        self._camera_cpub.publish(c_msg)

                    self._edited_frame_count += 1
                    self.get_logger().debug(
                        "Published {} frames. {} frames in queue.".format(
                            self._edited_frame_count,
                            self._edit_queue.qsize()))

            except queue.Empty:
                self.get_logger().debug("Frame buffer is empty")

            if (self._rec_state == RecordingState.Stopping and
                    self._edit_queue.qsize() == 0):
                self._rec_state = RecordingState.Stopped
                self.get_logger().info(
                    "Stopped recording after publishing {} frames. "
                    "Queue empty.".
                    format(self._edited_frame_count))

        if self._save_to_mp4:
            self.cv2_video_writer.release()
            self.get_logger().info(
                "Video written to {}.".format(
                    self._output_file_name_current))


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
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
