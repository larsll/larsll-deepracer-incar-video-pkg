#!/usr/bin/env python3

import logging
import importlib
import sys
from threading import Thread, Event

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from deepracer_interfaces_pkg.srv import SetLedCtrlSrv

from incar_video_pkg.constants import (
    LED_STATE_SRV, MONITOR_CHECK_TIME, RECORDING_STATE_SRV, PUBSUB_STATUS_TOPIC,
    LedColorMap, RecordingState)

from incar_video_interfaces_pkg.msg import StatusMsg
from incar_video_interfaces_pkg.srv import RecordStateSrv


class InCarVideoAutoCtrlNode(Node):
    """ This node is used to enable/disable the recording based on input
    from a ROS topic.
    """
    _shutdown = Event()
    _change_state = Event()
    _edit_node_status = StatusMsg()
    _target_edit_state = RecordingState.Stopped
    _current_led_color = LedColorMap.Black.value

    def __init__(self):
        super().__init__('incar_video_auto_ctrl_node')

        self.declare_parameter(
            'monitor_topic', '/inference_pkg/rl_results',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._monitor_topic = self.get_parameter('monitor_topic').value

        self.declare_parameter(
            'monitor_topic_type', 'deepracer_interfaces_pkg/msg/InferResultsArray',
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING))
        self._monitor_topic_type = self.get_parameter('monitor_topic_type').value
        module_name, class_name = self._monitor_topic_type.replace('/', '.').rsplit(".", 1)
        type_module = importlib.import_module(module_name)
        self._monitor_topic_class = getattr(type_module, class_name)

        self.declare_parameter(
            'monitor_topic_timeout', 1,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER))
        self._monitor_topic_timeout = self.get_parameter('monitor_topic_timeout').value
        self._monitor_last_received = self.get_clock().now()

        self.declare_parameter(
            'update_led', False,
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL))
        self._update_led = self.get_parameter('update_led').value

    def __enter__(self):

        # Subscription to receive status update from edit node.
        self._main_cbg = ReentrantCallbackGroup()
        self._edit_node_sub = self.create_subscription(
            StatusMsg, PUBSUB_STATUS_TOPIC, self._receive_status_callback, 1,
            callback_group=self._main_cbg)

        # Subscription to monitor topic.
        self._monitor_node_sub = self.create_subscription(
            self._monitor_topic_class, self._monitor_topic, self._receive_monitor_callback, 1,
            callback_group=self._main_cbg)

        # Service client to start and stop recording
        self._state_service_cli = self.create_client(
            RecordStateSrv, RECORDING_STATE_SRV)

        # Service client to change LED state
        self._setledstate_service_cli = self.create_client(
            SetLedCtrlSrv, LED_STATE_SRV)

        # Check if timeout receiver thread
        self._timeout_check_thread = Thread(target=self._timeout_check_thread_cb)
        self._timeout_check_thread.start()

        # Change monitor
        self._change_timer = self.create_timer(MONITOR_CHECK_TIME, callback=self._change_monitor_cb,
                                               callback_group=self._main_cbg)

        self.get_logger().info('Node started. Ready to control.')

        return self

    def __exit__(self, ExcType, ExcValue, Traceback):
        """Called when the object is destroyed.
        """
        self.get_logger().info('Stopping the node due to {}.'.format(ExcType.__name__))

        try:
            self._shutdown.set()
            self._check_rate.destroy()
            self._change_timer.destroy()
            self._timeout_check_thread.join()
        except:  # noqa E722
            self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))
        finally:
            self.get_logger().info('Node cleanup done. Exiting.')

    def _receive_status_callback(self, msg):
        """Receives the status updates from the edit node
        """
        if (self._edit_node_status.state == RecordingState.Running and
            self._target_edit_state == RecordingState.Running and
            (msg.state == RecordingState.Stopping or
             msg.state == RecordingState.Stopped)):
            self.get_logger().warn("Received inconsistent state from edit node. ({} | {} | {})"
                                   .format(msg.state, msg.published, msg.queue))
            self._target_edit_state = RecordingState.Stopped
        else:
            self.get_logger().debug("Received state from edit node. ({} | {} | {})"
                                    .format(msg.state, msg.published, msg.queue))
        self._edit_node_status = msg

        if (self._update_led):
            color = LedColorMap.Black.value
            if self._edit_node_status.state == RecordingState.Running:
                color = LedColorMap.Green.value
            elif self._edit_node_status.state == RecordingState.Stopping:
                color = LedColorMap.Orange.value
            else:
                color = LedColorMap.Red.value

            if (self._current_led_color != color):
                led_msg = SetLedCtrlSrv.Request(red=color[0], green=color[1], blue=color[2])
                _ = self._setledstate_service_cli.call_async(led_msg)
                self._current_led_color = color

    def _receive_monitor_callback(self, msg):
        """Permanent method that will receive commands via serial
        """
        try:
            self._monitor_last_received = self.get_clock().now()
            if self._edit_node_status.state == RecordingState.Stopped and \
                    self._target_edit_state == RecordingState.Stopped:
                self._target_edit_state = RecordingState.Running
                self._change_state.set()
                self.get_logger().info("Got callback from {}. Triggering start.". format(self._monitor_topic))

        except:  # noqa E722
            self.get_logger().error("{} occurred in _receive_monitor_callback.".format(sys.exc_info()[0]))

    def _timeout_check_thread_cb(self):
        try:
            self._check_rate = self.create_rate(5.0 / self._monitor_topic_timeout)
            timeout_duration = Duration(seconds=self._monitor_topic_timeout)

            while not self._shutdown.is_set():
                self._check_rate.sleep()
                dur_since_last_message = self.get_clock().now() - self._monitor_last_received

                if (dur_since_last_message > timeout_duration) and \
                        self._edit_node_status.state == RecordingState.Running and \
                        self._target_edit_state == RecordingState.Running:
                    self._target_edit_state = RecordingState.Stopped
                    self._change_state.set()
                    self.get_logger().info("Timeout. Triggering stop of recording.". format(self._monitor_topic))

        except:  # noqa E722
            self.get_logger().error("{} occurred in _timeout_check_thread_cb.".format(sys.exc_info()[0]))

    def _change_monitor_cb(self):
        """Permanent method that will monitor for change events
        """
        if not self._shutdown.is_set():
            try:
                if self._change_state.is_set():
                    self.get_logger().info("Changing state to {}"
                                           .format(self._target_edit_state.name))
                    resp = self._state_service_cli.call_async(
                        RecordStateSrv.Request(
                            state=self._target_edit_state.value))
                    self._change_state.clear()
            except:  # noqa E722
                self.get_logger().error("{} occurred.".format(sys.exc_info()[0]))


def main(args=None):

    try:
        rclpy.init(args=args)
        with InCarVideoAutoCtrlNode() as incar_video_auto_ctrl_node:
            executor = MultiThreadedExecutor()
            rclpy.spin(incar_video_auto_ctrl_node, executor)
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        incar_video_auto_ctrl_node.destroy_node()
    except KeyboardInterrupt:
        pass
    except:  # noqa: E722
        logging.exception("Error in Node")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
