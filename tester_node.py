import multiprocessing

# import pytz
import rclpy
import os
import logging
import json

from datetime import datetime
from rclpy.qos import QoSPresetProfiles, QoSReliabilityPolicy
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from time import monotonic, sleep

from industrial_ros.industrial_ros_node import IndustrialROS, IndustrialROSMode, NodeStatuses

from tester_mode_operation import TesterModeOperation



class IndustriesNodeTester(IndustrialROS, IndustrialROSMode):
    def __init__(self, node_suffix: int = 1):
        # super().__init__(node_suffix)
        IndustrialROS.__init__(self, node_suffix=node_suffix)
        IndustrialROSMode.__init__(self)

        self.status = NodeStatuses.READY
        self._node_type = "tester"

        self.mode_classes = {
            "oper": TesterModeOperation
        }
        _default_config = {
            "tester": {
                "mode": "oper"
            }
        }

        _default_config = json.dumps(_default_config)
        _config_msg = String()
        _config_msg.data = _default_config
        self.config_listener(_config_msg)
        self.initialize_inndustries_ros()


def main():
    rclpy.init()
    _tester_executor = MultiThreadedExecutor()
    _tester_node = IndustriesNodeTester(node_suffix=1)
    _tester_executor.add_node(_tester_node)
    _tester_executor.spin()
    _tester_node.destroy_node()
    rclpy.shutdown()
        
if __name__=="__main__":
    main()
