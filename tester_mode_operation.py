import rclpy
import logging
import json
import os
import numpy as np 

from industrial_ros.industrial_ros_node import IndustrialROS, IndustrialROSMode, NodeStatuses

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String
from time import monotonic, sleep
from threading import Timer
from collections import OrderedDict

# class TesterModeOperation(IndustrialROS, IndustrialROSMode):
#     def __init__(self, node_type, node_suffix: int = 1) -> None:
#         self.node_type = node_type
#         self.node_name = f"{node_type}_oper_{node_suffix}"
#         Node.__init__(self, node_name=self.node_name)
#         IndustrialROS.__init__(self, node_suffix=node_suffix)
#         IndustrialROSMode.__init__(self)
#         self.initialize_industries_ros()
#         logging.warning("Operation Mode Initialized")
#         self.mode_init = True

class TesterModeOperation(Node, IndustrialROSMode):
    def __init__(self, node_type, node_suffix: int = 1) -> None:
        # if not(hasattr(self, "node_name") and self.node_name is not None):
        #     self.node_name = f"{self.node_type}_{node_suffix}"
        #     super().__init__(node_name=self.node_name)
        #     # super().__init__()
        #     # Node.__init__(self, node_name=self.node_name)
        #     logging.warning(f"{self.__class__.__name__} initialized")
        # # self.initialize_industries_ros()
        self.node_name = f"{node_type}_oper_{node_suffix}"
        super().__init__(node_name=self.node_name)
        # self.initialize_industries_ros()
        logging.warning("Operation Mode Initialized")
        self.mode_init = True
    
    def start(self):
        for each in range(5):
            sleep(1)
            print("Oper Mode starting process", each)
            pass
        self.status = NodeStatuses.PLAYING
        # raise Exception("dummy error")
    
    def stop(self):
        for each in range(10):
            sleep(1)
            pass
        self.status = NodeStatuses.PAUSE
        # raise Exception("dummy error")

    
    def cleanup(self):
        self.command = None
        self.config_hash = None
        self.config = None
        self.destroy_node()
        

def main():
    rclpy.init()
    # _tester_executor = MultiThreadedExecutor()
    _tester_node = TesterModeOperation(node_type="tester",node_suffix=1)
    # _tester_executor.add_node(_tester_node)
    # _tester_executor.spin()
    rclpy.spin(_tester_node)
    _tester_node.destroy_node()
    rclpy.shutdown()
        
if __name__=="__main__":
    main()