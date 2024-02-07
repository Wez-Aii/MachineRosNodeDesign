import logging
import json
import time
import sys
from enum import Enum
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import String
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup


class Actuator(object):
    def __init__(self):
        pass

    def reset(self):
        pass

    def stop(self):
        pass

    def do_work(self):
        pass


class Sensor(object):
    def __init__(self):
        pass

    def reset(self):
        pass

    def stop(self):
        pass

    def get_value(self):
        pass


class NodeHeartbeats(Enum):
    OK = "ok"
    ERROR = "error"


class NodeStatuses(Enum):
    READY = 0
    PLAYING = 1
    PAUSE = 2
    WARN = 3
    ERROR = 4
    INPROGRESS = 5


class NodeCommands(Enum):
    START = "start"
    STOP = "stop"



class IndustrialROS(Node):
    """
    Concept
    Each ROS node represents a part of a bigger "Machine"
    Each of these representations will have a "system-level" bus communication via publisher/subscriber
    These elements will use "command" topic(bus) to receive commands and a "status" topic(bus) to send back current status
    Available Statuese: READY, PLAYING, PAUSE, WARN, ERROR, INPROGRESS
    Available Commands: START, STOP

    Each node will be self sufficient in terms of self sequencing
    """


    def __init__(self, node_suffix: int = 1):
        
        self._node_type: str
        self._node_name: str
        
        self.node_suffix = node_suffix

        self._control_callback_group = MutuallyExclusiveCallbackGroup()
        self._heartbeat_callback_group = ReentrantCallbackGroup()

        self._restart_wait_time = 3

        self._systemheartbeat_topic = "/systemheartbeat"
        self._configuration_topic = "/config"
        self._command_topic = "/command"
        self._feedback_topic = "/feedback"

        """
        Timer Setting to publish feedback
        """
        self._feedback_interval_sec = 5

        """
        Timeout to check if the last heartbeat has been too long
        """
        self._heartbeat_timeout_sec = 60

        """
        These variables represent current statues/messages
        """
        self.heartbeat = time.monotonic()
        self.command = None
        self.config_hash = None
        self.config = None
        self.status = None

        self.error = None 
        self.warning = None 
        self.info = None

    def initialize_inndustries_ros(self):
        self._heartbeat_subscriber = self.create_subscription(
            String,
            self._systemheartbeat_topic,
            self.heartbeat_listener,
            QoSPresetProfiles.get_from_short_key("services_default"),
            callback_group=self._heartbeat_callback_group
        )
        
        self._config_subscriber = self.create_subscription(
            String,
            self._configuration_topic,
            self.config_listener,
            QoSPresetProfiles.get_from_short_key("services_default"),
            callback_group=self._control_callback_group
        )
        
        self._command_subscriber = self.create_subscription(
            String,
            self._command_topic,
            self.command_listener,
            QoSPresetProfiles.get_from_short_key("services_default"),
            callback_group=self._control_callback_group
        )
        
        self._feedback_publisher = self.create_publisher(
            String,
            self._feedback_topic,
            QoSPresetProfiles.get_from_short_key("service_default")
        )

        self._feedback_timer = self.create_timer(
            self._feedback_interval_sec,
            self._feedback_timer_callback,
            callback_group=self._control_callback_group
        )

    def heartbeat_listener(self, command: String):
        pass

    def config_listener(self):
        pass

    def command_listener(self, command: String):
        pass

    def check_heartbeat_timeout(self):
        pass

    def _feedback_timer_callback(self):
        pass



