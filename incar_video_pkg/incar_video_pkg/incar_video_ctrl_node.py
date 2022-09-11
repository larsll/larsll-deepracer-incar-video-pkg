#!/usr/bin/env python3

import logging
from xxlimited import new
from serial import Serial, SerialException
import sys
from threading import Thread, Event

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from deepracer_interfaces_pkg.srv import VideoStateSrv, SetLedCtrlSrv

from incar_video_pkg.constants import (
    LED_SET_SERVICE_NAME, RECORDING_STATE_SERVICE_NAME, STATUS_TOPIC,
    MAX_FRAMES_IN_QUEUE, QUEUE_WAIT_TIME, VIDEO_STATE_SRV, LedColorMap, RecordingState)
from incar_video_pkg.utils import DoubleBuffer

from incar_video_interfaces_pkg.msg import StatusMsg
from incar_video_interfaces_pkg.srv import RecordStateSrv


class InCarVideoCtrlNode(Node):
    """ This node is used to produce frames for the AWS kinesis video stream
    and for saving the mp4 and uploading to S3. Both are subscribed to the
    output of the image topic produced by this node.
    """
    _shutdown = Event()
    _change_state = Event()
    _edit_node_status = StatusMsg()
    _target_edit_state = RecordingState.Stopped

    def __init__(self):
        super().__init__('incar_video_ctrl_node')

        self.declare_parameter(
            'serial_port', '/dev/ttyS0',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))

        self._serial_port = self.get_parameter('serial_port').value

        self.declare_parameter(
            'update_led', True,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))

        self._update_led = self.get_parameter('update_led').value

    def __enter__(self):
        self._edit_node_sub_cbg = ReentrantCallbackGroup()
        self._edit_node_sub = self.create_subscription(
            StatusMsg, STATUS_TOPIC, self._receive_status_callback, 1,
            callback_group=self._edit_node_sub_cbg)

        # Call ROS service to enable the Video Stream
        self._camera_state_cli = self.create_client(
            VideoStateSrv, VIDEO_STATE_SRV)
        while not self._camera_state_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Camera service not available, waiting...')

        self.get_logger().info("Camera service available, enabling video" +
                               "stream.")
        _ = self._camera_state_cli.call_async(
            VideoStateSrv.Request(activate_video=1))

        # Service client to start and stop recording
        self._state_service_cli = self.create_client(
            RecordStateSrv, RECORDING_STATE_SERVICE_NAME)

        # Service client to change LED state
        self._setledstate_service_cli = self.create_client(
            SetLedCtrlSrv, LED_SET_SERVICE_NAME)

        # Serial receiver thread
        self._serial_receive_thread = Thread(target=self._serial_receive_cb)
        self._serial_receive_thread.start()

        # Change thread
        self._change_thread = Thread(target=self._change_cb)
        self._change_thread.start()

        self.get_logger().info('Node started. Ready to control.')

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'.format(ExcType.__name__))

        try:
            self._shutdown.set()
            self._serial_receive_thread.join()
            self._change_thread.join()
        except:  # noqa E722
            self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))
        finally:
            self.get_logger().info('Node cleanup done. Exiting.')

    def _receive_status_callback(self, msg):
        """Receives the status updates from the edit node
        """
        self._edit_node_status = msg
        self.get_logger().debug("Received message from edit node. ({} | {} | {})"
                                .format(msg.state, msg.published, msg.queue))

        if (self._update_led):
            color = LedColorMap.Black.value
            if self._edit_node_status.state == RecordingState.Running:
                color = LedColorMap.Green.value
            elif self._edit_node_status.state == RecordingState.Stopping:
                color = LedColorMap.Orange.value
            else:
                color = LedColorMap.Red.value

            led_msg = SetLedCtrlSrv.Request(red=color[0], green=color[1], blue=color[2])
            _ = self._setledstate_service_cli.call(led_msg)

    def _serial_receive_cb(self):
        """Permanent method that will receive commands via serial
        """
        try:
            with Serial(
                    port=self._serial_port,
                    baudrate=9600,
                    timeout=1) as serial:
                while serial.is_open and not self._shutdown.isSet():
                    serial_in_b = serial.readline()
                    serial_in_str = str(serial_in_b, encoding="ascii")[:1]
                    if serial_in_str != '':
                        try:
                            new_cmd = int(serial_in_str)
                            self.get_logger().debug("Serial input {} as int {}"
                                                    .format(serial_in_b, new_cmd))

                            if new_cmd != self._edit_node_status.state:
                                self._target_edit_state = RecordingState(new_cmd)
                                self._change_state.set()

                        except ValueError:
                            self.get_logger().warn("Serial input {} not int"
                                                   .format(serial_in_b))
                    else:
                        self.get_logger().debug("Serial read timeout. No data.")
        except SerialException:
            self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))
        except:  # noqa E722
            self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))

    def _change_cb(self):
        """Permanent method that will monitor for change events
        """

        while not self._shutdown.is_set():
            try:
                if self._change_state.wait(timeout=1.0):
                    self.get_logger().info("Changing state to {}"
                                           .format(self._target_edit_state.name))
                    resp = self._state_service_cli.call(
                        RecordStateSrv.Request(
                            state=self._target_edit_state.value))
                    self._change_state.clear()
            except:  # noqa E722
                self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))


def main(args=None):

    try:
        rclpy.init(args=args)
        with InCarVideoCtrlNode() as incar_video_ctrl_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(incar_video_ctrl_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        incar_video_ctrl_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
