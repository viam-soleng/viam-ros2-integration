#!/usr/bin/python3

# TODO: handler does not seem to execute properly
import asyncio
import os
import signal
import sys

from viam.components.base import Base
from viam.components.camera import Camera
from viam.components.movement_sensor import MovementSensor
from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.module.module import Module
from components import RosBase, RosImu, RosLidar, RosSensor, ViamRosNode, RosCamera
from utils import RclpyNodeManager

logger = getLogger(__name__)

rclpy_mgr = None
viam_node = None


def sigterm_handler(_signo, _stack_frame):
    """
    might not be needed

    :param _signo:
    :param _stack_frame:
    :return:
    """
    logger.info('attempting rclpy shutdown')
    sys.exit(0)

    
async def main(addr: str) -> None:
    try:
        global rclpy_mgr
        global viam_node
        logger.info('starting ros2 module server')

        # setup viam ros node & do we need to do work in finally
        nodename = ''
        namespace = ''

        if 'VIAM_NODE_NAME' in os.environ and os.environ['VIAM_NODE_NAME'] != '':
            nodename = os.environ['VIAM_NODE_NAME']
        else:
            raise Exception('VIAM_NODE_NAME environment variable required, see INSTALL.md')

        if 'VIAM_ROS_NAMESPACE' in os.environ and os.environ['VIAM_ROS_NAMESPACE'] != '':
            namespace = os.environ['VIAM_ROS_NAMESPACE']

        rclpy_mgr = RclpyNodeManager.get_instance()
        viam_node = ViamRosNode.get_viam_ros_node(node_name=nodename, namespace=namespace)
        rclpy_mgr.spin_and_add_node(viam_node)

        m = Module(addr)
        m.add_model_from_registry(Base.SUBTYPE, RosBase.MODEL)
        m.add_model_from_registry(MovementSensor.SUBTYPE, RosImu.MODEL)
        m.add_model_from_registry(Camera.SUBTYPE, RosLidar.MODEL)
        m.add_model_from_registry(Sensor.SUBTYPE, RosSensor.MODEL)
        m.add_model_from_registry(Camera.SUBTYPE, RosCamera.MODEL)
        await m.start()
    except Exception as e:
        raise Exception(f'Error occured starting module: {e}')
    finally:
        rclpy_mgr.shutdown()


if __name__ == '__main__':
    if len(sys.argv) != 2:
        raise Exception('need socket path as cmd line arg')
    signal.signal(signal.SIGTERM, sigterm_handler)
    asyncio.run(main(sys.argv[1]))
