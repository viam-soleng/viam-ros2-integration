"""
the ViamRosNode will represent the node that will publish and subscribe to
ROS nodes as well as manage other aspects of ROS in the future.

Ideally we will have a single ROS node but this will change based on the need
of the architectures we come across
"""
from rclpy.node import Node
import os
from typing_extensions import Self
from utils import RclpyNodeManager


class ViamRosNode(Node):
    """
    ViamRosNode
    A single node to integrate with the ROS robot
    """
    node = None

    @classmethod
    def get_viam_ros_node(
        cls,
        namespace: str = '',
        node_name: str = '',
        enable_rosout: bool = True
    ) -> Self:
        """
        get_viam_ros_node attempts to get an existing ros node and if one
        does not exist it will create it
        """
        if cls.node is None:
            # this check is also done in main.py - leave here for testing purposes
            if node_name is None or node_name == '':
                if 'VIAM_NODE_NAME' in os.environ and os.environ['VIAM_NODE_NAME'] != '':
                    node_name = os.environ['VIAM_NODE_NAME']
                else:
                    raise Exception(
                        'cannot find valid viam node name in environment,'
                        ' please set VIAM_NODE_NAME\nsee install instructions'
                    )
            cls.node = ViamRosNode(namespace, node_name, enable_rosout)
        return cls.node

    def __init__(
        self,
        namespace: str = '',
        node_name: str = '',
        enable_rosout: bool = True
    ) -> None:
        """
        __init__ creates the node
        """
        super().__init__(node_name, namespace=namespace, enable_rosout=enable_rosout)
        rclpy_mgr = RclpyNodeManager.get_instance()
        rclpy_mgr.spin_and_add_node(self)
        self.get_logger().debug('created base node')
