#!/usr/bin/env python3

import logging
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from sensor_msgs.msg import CompressedImage as ROSCImg
from sensor_msgs.msg import Imu

from deepracer_interfaces_pkg.srv import VideoStateSrv

from incar_video_interfaces_pkg.msg import EvoSensorMsg as EvoSensorImuMsg

from incar_video_pkg.constants import (
    PUBSUB_SENSOR_TOPIC, SUBSCRIBE_IMAGE_TOPIC, SUBSCRIBE_IMU_TOPIC, CAMERA_STATE_SRV)
from incar_video_pkg.utils import DoubleBuffer


class InCarVideoSensorFusionNode(Node):
    """ This node is used to fuse sensor values as a pre-step to the video editing.
        To avoid altering the original SensorFusion node, this will run in-process to the edit node.
    """

    def __init__(self, node_name):
        super().__init__(node_name)

        self.declare_parameter('enable_camera_src', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))
        self._enable_camera_src = self.get_parameter('enable_camera_src').value

        self.declare_parameter('enable_imu', False, ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL))
        self._enable_imu = self.get_parameter('enable_imu').value

    def __enter__(self):

        self._main_cbg = ReentrantCallbackGroup()

        # Subscribe to camera
        self._camera_sub = self.create_subscription(
            ROSCImg, SUBSCRIBE_IMAGE_TOPIC, self._receive_camera_frame_callback, 5,
            callback_group=self._main_cbg)

        # Call ROS service to enable the Video Stream
        if self._enable_camera_src:
            self._camera_state_cli = self.create_client(VideoStateSrv, CAMERA_STATE_SRV)
            while not self._camera_state_cli.wait_for_service(timeout_sec=5.0):
                self.get_logger().info('Camera service not available, waiting...')

            self.get_logger().info("Camera service available, enabling video"
                                   " stream.")
            _ = self._camera_state_cli.call_async(VideoStateSrv.Request(activate_video=1))

        # Subscribe to IMU
        if self._enable_imu:
            self._imu_buffer = DoubleBuffer()
            self._camera_sub = self.create_subscription(
                Imu, SUBSCRIBE_IMU_TOPIC, self._receive_imu_callback, 5,
                callback_group=self._main_cbg)

        # Publisher to republish the fused sensor stream.
        self._sensor_pub = self.create_publisher(EvoSensorImuMsg, PUBSUB_SENSOR_TOPIC, 10,
                                                 callback_group=self._main_cbg)

        self.get_logger().info('Node started.')

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'
                               .format(ExcType.__name__))

        self.get_logger().info('Node cleanup done. Exiting.')

    def _receive_camera_frame_callback(self, frame):
        """ Callback for the main input. Once a new image is received, it will be fused with the
        latest IMU data (if enabled), and the message is published.

        Arguments:
            frame (sensor_msgs/CompressedImage): Image/Sensor topic of the camera image frame
        """
        if rclpy.ok():
            out_msg = EvoSensorImuMsg()
            out_msg.images.append(frame)

            if self._enable_imu:
                try:
                    out_msg.imu_data = self._imu_buffer.get(block=False)
                except DoubleBuffer.Empty:
                    pass

            self._sensor_pub.publish(out_msg)

    def _receive_imu_callback(self, frame):
        """ Callback for the IMU input. Once a new message is received, it will be put to the
        double buffer and read with the next message.

        Arguments:
            frame (sensor_msgs/Imu): Sensor topic of the imu sensor
        """
        if rclpy.ok():
            self._imu_buffer.put(frame)


def main(args=None):

    try:
        rclpy.init(args=args)
        with InCarVideoSensorFusionNode() as incar_video_sensor_fusion_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(incar_video_sensor_fusion_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        incar_video_sensor_fusion_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
