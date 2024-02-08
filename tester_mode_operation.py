import rclpy
import logging
import json
import os
import numpy as np 

from industrial_ros.industrial_ros_node import IndustrialROS, IndustrialROSMode, NodeStatuses

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String
from time import monotonic, sleep
from threading import Timer
from collections import OrderedDict

class TesterModeOperation(Node, IndustrialROSMode):
    def __init__(self, node_suffix: int = 1) -> None:
        self._node_name = f"{self._node_type}_{node_suffix}"
        super().__init__(node_name=self._node_name)
        logging.warning(f"{self.__class__.__name__} initialized")
        logging.warning("Operation Mode Initialized")
        self.mode_init = True
    
    def start(self):
        for each in range(10):
            sleep(1)
            pass
        self.status = NodeStatuses.PLAYING
        # if each > 8:
        raise Exception("dummy error")
    
    def stop(self):
        for each in range(10):
            sleep(1)
            pass
        self.status = NodeStatuses.PLAYING
        # if each > 8:
        raise Exception("dummy error")
        