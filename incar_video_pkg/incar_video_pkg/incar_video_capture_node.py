#!/usr/bin/env python3
import logging

import rclpy
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import Imu

from incar_video_pkg.logger import Logger
from incar_video_pkg.utils import DoubleBuffer
from incar_video_pkg import constants
from incar_video_pkg.constants import Mp4Parameter

from deepracer_interfaces_pkg.srv import VideoStateSrv
from deepracer_interfaces_pkg.msg import EvoSensorMsg, CameraMsg


LOG = Logger(__name__, logging.INFO).get_logger()


class InCarVideoCaptureNode(Node):
    """ This node is used to produce frames for the AWS kinesis video stream and
    for saving the mp4 and uploading to S3. Both are subscribed to the output of
    the image topic produced by this node.
    """

    def __init__(self):
        super().__init__('incar_video_capture_node')
       

        # Duplicate frames -- if no new image frame is available then push the previous one.
        self.declare_parameter('duplicate_frame', True, ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self._duplicate_frame = self.get_parameter('duplicate_frame').value

        # Fetching main camera frames, start consumer thread and producer thread for main camera frame
        self.main_camera_topic = constants.MAIN_CAMERA_TOPIC
        self.imu_topic = constants.IMU_TOPIC
        self.last_publish = self.get_clock().now()
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
        )


        # Publisher to broadcast the notification messages.
        self.stream_cbg = ReentrantCallbackGroup()
        self.stream_pub = self.create_publisher(EvoSensorMsg,
                                  constants.PUBLISH_SENSOR_TOPIC,
                                  10)

    def __enter__(self):

        # Call ROS service to enable the Video Stream
        self.cli = self.create_client(VideoStateSrv, constants.VIDEO_STATE_SRV)
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Camera service not available, waiting...')

        self.get_logger().info("Camera service available, enabling video stream.")
        req = VideoStateSrv.Request()
        req.activate_video = 1
        _ = self.cli.call_async(req)

        self._camera_data_buffer = DoubleBuffer(clear_data_on_get=self._duplicate_frame)
        self.camera_sub_cbg = ReentrantCallbackGroup()
        self.camera_sub = self.create_subscription(CameraMsg,
                                 self.main_camera_topic,
                                 self._producer_frame_thread,
                                 10,
                                 callback_group=self.camera_sub_cbg)

        self._imu_data_buffer = DoubleBuffer(clear_data_on_get=False)
        self.imu_sub_cbg = ReentrantCallbackGroup()
        self.imu_sub = self.create_subscription(Imu,
                                 self.imu_topic,
                                 self._update_imu_data,
                                 1,
                                 callback_group=self.imu_sub_cbg)


        self.timer = self.create_timer(1/Mp4Parameter.FPS.value,self._timer_processor)

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Exiting.')

    def _update_imu_data(self, imu_data):
        """ Used to update the racers metric information
        """
        self._imu_data_buffer.put(imu_data)

    def _producer_frame_thread(self, frame):
        """ Callback for the main camera frame. Once a new image is received, a new message is
            created that contains the latest IMU data together with the picture.

        Arguments:
            frame (deepracer_interfaces_pkg.msg.CameraMsg): Image topic of the camera image frame
        """

        try:
            sensor_data = EvoSensorMsg()
            sensor_data.images = frame.images
            sensor_data.imu_data = self._imu_data_buffer.get(block=False)

            self._camera_data_buffer.put(sensor_data)

            LOG.debug(f"Frame to buffer: { Time.from_msg(sensor_data.images[0].header.stamp).nanoseconds / 1e9 }")
            
        except DoubleBuffer.Empty:
            LOG.warn('No Camera Message and/or IMU Message available.')
            pass

    def _timer_processor(self):
        try:
            sensor_data = self._camera_data_buffer.get(block=True)
            self.stream_pub.publish(sensor_data)
            current_publish = Time.from_msg(sensor_data.images[0].header.stamp)
            LOG.debug(f"Publishing frame: { current_publish.nanoseconds / 1e9 }")
            if current_publish == self.last_publish:
                LOG.warn(f"Publishing duplicate frame: { current_publish.nanoseconds / 1e9 }")
            self.last_publish = current_publish

        except:
            LOG.error('Error in timer process.')

def main(args=None):

    try:
        rclpy.init(args=args)
        with InCarVideoCaptureNode() as incar_video_capture_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(incar_video_capture_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        incar_video_capture_node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
