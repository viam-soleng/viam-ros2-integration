"""
sensor.py

We treat all non-specific topics as sensors right now
"""

import importlib
import numpy as np
import threading

import rclpy
from array import array
from logging import Logger
from threading import Lock
from utils import RclpyNodeManager
from typing import Any, ClassVar, Mapping, Optional, Sequence
from typing_extensions import Self
from viam.components.sensor import Sensor
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from .viam_ros_node import ViamRosNode

logger: Logger = getLogger(__name__)


class RosSensor(Sensor, Reconfigurable):
    """
    The ROS sensor can represent any ROS2 message

    The sensor takes the message type as well as the python module the
    message type is in

    The sensor will then use the standard message functions to dynamically
    build retrieve the results
    """

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-soleng", "ros2"), "sensor")
    ros_topic: str
    ros_node: ViamRosNode
    ros_msg_type: str
    ros_sensor_cls: Any
    ros_msg_package: str
    lock: threading.Lock
    msg: Any
    subscription: rclpy.subscription.Subscription

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """
        Class method to create a new instance of the sensor
        """
        sensor = cls(config.name)
        sensor.ros_node = None
        sensor.reconfigure(config, dependencies)
        return sensor

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> tuple[Sequence[str], Sequence[str]]:
        """
        class method used to validate the configuration of the sensor

        For a sensor to be setup correctly, we require the following attributes:
        1. ros_topic: this will be the topic we listen to for messages
        2. ros_msg_package: this will be where the module is located for the message
        3. ros_msg_type: this is the class that represents the ROS message
        """
        topic = config.attributes.fields["ros_topic"].string_value
        msg_package = config.attributes.fields["ros_msg_package"].string_value
        msg_type = config.attributes.fields["ros_msg_type"].string_value

        if topic == "":
            raise Exception("ros_topic required")

        if msg_package == "":
            raise Exception("ros_msg_package required")
        if msg_type == "":
            raise Exception("ros_msg_type required")

        try:
            tmp = importlib.import_module(msg_package)
            if not hasattr(tmp, msg_type):
                raise Exception(
                    f"invalid ros_msg_type, {msg_package} does not have {msg_type}"
                )
        except ModuleNotFoundError as mnfe:
            raise Exception(f"invalid ros_msg_type: {mnfe}")

        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        """
        This method is called with the robot configuration is changed, for this process
        to work we will destroy the subscription as well as reset the node then setup
        the subscription with the topic
        """
        self.ros_topic = config.attributes.fields["ros_topic"].string_value
        self.ros_msg_package = config.attributes.fields["ros_msg_package"].string_value
        self.ros_msg_type = config.attributes.fields["ros_msg_type"].string_value

        lib = importlib.import_module(self.ros_msg_package)
        self.ros_sensor_cls = getattr(lib, self.ros_msg_type)

        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.subscription = self.ros_node.create_subscription(
            self.ros_sensor_cls,
            self.ros_topic,
            self.subscriber_callback,
            qos_profile=qos_policy,
        )
        self.lock = Lock()
        self.msg = None

        self.ros_node = ViamRosNode.get_viam_ros_node()

        rcl_mgr = RclpyNodeManager.get_instance()
        rcl_mgr.remove_node(self.ros_node)
        rcl_mgr.spin_and_add_node(self.ros_node)

    def subscriber_callback(self, msg):
        """
        callback for the subscriber
        """
        self.msg = msg

    async def get_readings(
        self,
        *,
        extra: Optional[Mapping[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, Any]:
        """
        get_readings is the core sensor interface which is called by
        the data capture service as well as other components

        When working in the ROS pub/sub model, if the publisher is not
        ready we will return a default 'NOT_READY' message.

        :param extra:
        :param timeout:
        :param kwargs:
        :return:
        """
        with self.lock:
            msg = self.msg

        if msg is not None:
            return build_msg(msg)
        return {"value": "NOT_READY"}


def build_msg(msg):
    """
    build_msg() will recursively grab all values from a ROS message

    Every ROS message custom or standard has the "get_fields_and_field_types"
    method which we can use to recursively build a python dictionary representing
    the ROS message.

    The base case for this algorithm is when we get to the simple python types:
    numbers, strings, etc.


    :param msg:
    :return:
    """
    r_data = {}
    if hasattr(msg, "get_fields_and_field_types"):
        fields_and_types = msg.get_fields_and_field_types()
        for key in fields_and_types.keys():
            r_data[key] = build_msg(getattr(msg, key))
    else:
        # builtin type
        msg_type = type(msg)
        # for list types we must analyze each element as it could be a ROS type
        # which needs further decomposition
        if (
            msg_type is list
            or msg_type is tuple
            or msg_type is set
            or msg_type is array
            or msg_type is np.ndarray
        ):
            l_data = []
            for value in msg:
                l_data.append(build_msg(value))
            return l_data
        # for dictionary types we must analyze each value found in the key as it can
        # be a ROS type or list type which will need further decomposition
        elif msg_type is dict:
            d_data = {}
            for key in msg.keys():
                d_data[key] = build_msg(msg[key])
            return d_data
        else:
            return msg
    return r_data


"""
Register the new MODEL as well as define how the object is validated 
and created
"""
Registry.register_resource_creator(
    Sensor.API,
    RosSensor.MODEL,
    ResourceCreatorRegistration(RosSensor.new, RosSensor.validate_config),
)
