import multiprocessing

from rclpy.node import Node

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
from tester_mode_test import TesterModeTesting


# class IndustriesNodeTester(IndustrialROS):
#     def __init__(self, node_suffix: int = 1):
#         super().__init__(node_suffix)
#         self.status = NodeStatuses.READY
#         self.node_type = "tester"
#         self.node_name = f"{self.node_type}_oper_{node_suffix}"
#         Node.__init__(self, node_name=self.node_name)
#         self.initialize_industries_ros()
#         logging.warning("Base Node Initialized")
#         self.mode_init = True

#     def destroy_ros(self):
#         pass

#     def cleanup(self):
#         pass

#     def apply_config(self):
#         pass

#     def start(self):
#         pass

#     def stop(self):
#         pass

class IndustriesNodeTester(IndustrialROS, IndustrialROSMode):
    def __init__(self, node_suffix: int = 1):
        # super().__init__(node_suffix)
        IndustrialROS.__init__(self, node_suffix=node_suffix)
        IndustrialROSMode.__init__(self)


        self.status = NodeStatuses.READY
        self.node_type = "tester"

        self.mode_classes = {
            "oper": TesterModeOperation,
            "test": TesterModeTesting
        }
        _default_config = {
            "tester": {
                "mode": "oper",
                # "mode": "test"
            }
        }

        _default_config = json.dumps(_default_config)
        _config_msg = String()
        _config_msg.data = _default_config
        self.config_listener(_config_msg)
        # self.initialize_industries_ros()


def main():
    rclpy.init()
    # _tester_executor = MultiThreadedExecutor()
    _tester_node = IndustriesNodeTester(node_suffix=1)
    # _tester_executor.add_node(_tester_node)
    # _tester_executor.spin()
    rclpy.spin(_tester_node)
    _tester_node.destroy_node()
    rclpy.shutdown()
        
if __name__=="__main__":
    main()
