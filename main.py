#!/usr/bin/python3

import asyncio
import os
import signal
import sys
from typing import Union
from viam.components.base import Base
from viam.components.camera import Camera
from viam.components.generic import Generic
from viam.components.movement_sensor import MovementSensor
from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.module.module import Module

from components import (
    RosBase,
    RosCamera,
    RosImu,
    RosLidar,
    RosSensor,
    RosTopicPublisher,
    ViamRosNode,
)

from services.ros2_logger import MyROS2LoggerService, ROS2LoggerService
from services.ros2_action import MyROS2ActionService, ROS2ActionService
from services.ros2_service import MyROS2ServiceService, ROS2ServiceService

from utils import RclpyNodeManager

logger = getLogger(__name__)

rclpy_mgr: Union[RclpyNodeManager, None] = None
viam_node = None


def sigterm_handler(_signo, _stack_frame):
    """

    :param _signo:
    :param _stack_frame:
    :return:
    """
    logger.info("attempting rclpy shutdown")
    sys.exit(0)


async def main(addr: str) -> None:
    global rclpy_mgr
    global viam_node
    logger.info("starting ros2 module server")
    try:
        for key in os.environ.keys():
            logger.debug(f"ENV: {key} {os.environ[key]}")

        # setup viam ros node & do we need to do work in finally
        node_name: str = "VIAM_NODE"
        namespace: str = ""

        if "VIAM_NODE_NAME" in os.environ and os.environ["VIAM_NODE_NAME"] != "":
            node_name = os.environ["VIAM_NODE_NAME"]
        else:
            logger.info(f"Using default VIAM_NODE_NAME of {node_name}")

        if (
            "VIAM_ROS_NAMESPACE" in os.environ
            and os.environ["VIAM_ROS_NAMESPACE"] != ""
        ):
            namespace = os.environ["VIAM_ROS_NAMESPACE"]

        rclpy_mgr = RclpyNodeManager.get_instance()
        viam_node = ViamRosNode.get_viam_ros_node(
            node_name=node_name, namespace=namespace
        )
        rclpy_mgr.spin_and_add_node(viam_node)

        m = Module(addr)
        m.add_model_from_registry(Base.API, RosBase.MODEL)
        m.add_model_from_registry(MovementSensor.API, RosImu.MODEL)
        m.add_model_from_registry(Camera.API, RosLidar.MODEL)
        m.add_model_from_registry(Sensor.API, RosSensor.MODEL)
        m.add_model_from_registry(Generic.API, RosTopicPublisher.MODEL)
        m.add_model_from_registry(Camera.API, RosCamera.MODEL)
        m.add_model_from_registry(ROS2LoggerService.API, MyROS2LoggerService.MODEL)
        m.add_model_from_registry(ROS2ActionService.API, MyROS2ActionService.MODEL)
        m.add_model_from_registry(ROS2ServiceService.API, MyROS2ServiceService.MODEL)
        await m.start()
    except Exception as e:
        raise Exception(f"Error occurred starting module: {e}")
    finally:
        if rclpy_mgr is not None:
            rclpy_mgr.shutdown()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        raise Exception("need socket path as cmd line arg")
    signal.signal(signal.SIGTERM, sigterm_handler)
    asyncio.run(main(sys.argv[1]))
