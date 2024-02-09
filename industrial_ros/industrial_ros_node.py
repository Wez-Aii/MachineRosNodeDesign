import logging
import json
import time
import sys
from enum import Enum
import rclpy
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
        
        self.node_type: str
        self.node_name: str

        self.mode_classes : dict
        
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
        self.mode = None
        self.command = None
        self.config_hash = None
        self.config = None
        self.status = None

        self.error = None 
        self.warning = None 
        self.info = None

    def initialize_industries_ros(self):
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
            QoSPresetProfiles.get_from_short_key("services_default")
        )

        self._feedback_timer = self.create_timer(
            self._feedback_interval_sec,
            self._feedback_timer_callback,
            callback_group=self._control_callback_group
        )

    def heartbeat_listener(self, heartbeat: String):
        """
        heartbeat will be message of "OK/ERROR"
        """
        if heartbeat.data == NodeHeartbeats.ERROR.value:
            if self.status == NodeStatuses.ERROR:
                logging.critical("Func(heartbeat_listener_ - System Error: Restarting in 3 seconds")
                self._feedback_timer_callback()
                # time.sleep(self._restart_wait_time)
                """
				Restart ROS Service:
				Only when the node itself was error, ROS service needs to shutdown by itself 
				"""
                # logging.error(f"Node Error - {self.error}")
                # sys.exit(f"Node Error - {self.error}")
            else:
                self.heartbeat = time.monotonic()
        else:
            self.heartbeat = time.monotonic()

    def config_listener(self, config: String):
        _config = config.data
        _config = json.loads(_config)
        _config = _config.get(self.node_type, {})
        self.config = _config
        _mode = _config.get("mode")
        _config_string = json.dumps(self.config)

        if (_mode != self.mode and _mode in self.mode_classes) or self.config_hash != hash(_config_string):
            try:
                self.stop() # type: ignore
            except Exception as e:
                logging.error(str(e))
                self.status = NodeStatuses.ERROR
                self.error = str(e)
                self._feedback_timer_callback()
            self.cleanup() # type: ignore
            # rclpy.shutdown()
            self.mode = _mode
            # Change base classes, so the "self" can have access to new methods
            self.__class__.__bases__ = (IndustrialROS, self.mode_classes.get(_mode))
            self.mode_classes.get(_mode).__init__(self, node_type=self.node_type, node_suffix=self.node_suffix)
            self.initialize_industries_ros()
        
        self.config_hash = hash(_config_string)
        try:
            self.apply_config(_config_string) # type: ignore
        except Exception as e:
            self.error = str(e)
            self.status = NodeStatuses.ERROR
            self._feedback_timer_callback()


    def command_listener(self, command: String):
        _command_msg = command.data
        _command_msg = json.loads(_command_msg)
        _target_node = _command_msg.get("node_name")
        _command = _command_msg.get("command")

        if self.mode_init:
            if self.node_name == _target_node or _target_node == "all":
                try:
                    if self.command != _command:
                        self.status = NodeStatuses.INPROGRESS
                        self.command = _command
                        if self.command == NodeCommands.START.value:
                            self.start() # type: ignore
                        elif self.command == NodeCommands.STOP.value:
                            self.stop() # type: ignore
                        else:
                            pass
                    else:
                        if self.command == NodeCommands.START.value and self.status != NodeStatuses.PLAYING:
                            self.start() # type: ignore
                        elif self.command == NodeCommands.STOP.value and self.status not in [NodeStatuses.PAUSE, NodeStatuses.READY]:
                            self.stop() # type: ignore
                        else:
                            pass
                except Exception as e:
                    logging.error(str(e))
                    self.status = NodeStatuses.ERROR
                    self.error = str(e)
                    self._feedback_timer_callback()
        

    def check_heartbeat_timeout(self):
        if time.monotonic() - self.heartbeat > self._heartbeat_timeout_sec:
            logging.critical("System Heartbeat Not Detected.....")
            logging.critical(f"Restarting in {self._restart_wait_time} seconds")
            time.sleep(self._restart_wait_time)
            sys.exit("System Heartbeat Not Detected")

    def _feedback_timer_callback(self):
        feedback = {
            "node_type": self.node_type,
            "node_name": self.get_name(),
            "mode": self.mode,
            "command": self.command,
            "config": self.config_hash,
            "status": self.status.value if self.status is not None else None,
            "info": self.info,
            "warning": self.warning,
            "error": self.error
        }
        feedback = json.dumps(feedback)
        _feedback = String()
        _feedback.data = feedback
        self._feedback_publisher.publish(_feedback)
        # self.check_heartbeat_timeout()


class IndustrialROSMode:
    def __init__(self) -> None:
        self.mode_init = False
        pass

    def post_initialize_ros(self):
        pass

    def destroy_ros(self):
        pass

    def cleanup(self):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def apply_config(self, config: str):
        pass
