import logging
from threading import Lock
import viam
from std_msgs.msg import String
from utils import RclpyNodeManager
from viam.logging import getLogger
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.generic import Generic
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.timer import Rate
from geometry_msgs.msg import Twist
from .viam_ros_node import ViamRosNode

logger = getLogger(__name__)


class RosTopicPublisher(Generic, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'ros2'), 'topic-publisher')
    lock: Lock
    logger: logging.Logger
    msg: String
    publisher: Publisher
    publish_rate: float
    rate: Rate
    ros_node: ViamRosNode
    ros_topic: str

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        publisher = cls(config.name)
        publisher.ros_node = None
        publisher.reconfigure(config, dependencies)
        return publisher

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        topic = config.attributes.fields['ros_topic'].string_value
        publish_rate = float(config.attributes.fields['publish_rate'].string_value)

        if topic == '':
            raise Exception('ros_topic required')

        if publish_rate == 0.0:
            raise Exception('rate required')

        return []

    def ros_publisher_cb(self):
        self.publisher.publish(self.msg)


    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.publish_rate = float(config.attributes.fields['publish_rate'].string_value)
        self.msg = String()

        if self.ros_node is not None:
            if self.publisher is not None:
                self.ros_node.destroy_publisher(self.publisher)
            if self.rate is not None:
                self.ros_node.destroy_rate(self.rate)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        self.publisher = self.ros_node.create_publisher(String, self.ros_topic, 10)
        self.rate = self.ros_node.create_timer(self.publish_rate, self.ros_publisher_cb)
        self.lock = Lock()

    async def do_command(
            self,
            command: Mapping[str, ValueTypes],
            *,
            timeout: Optional[float] = None,
            **kwargs
    ):
        for key in command:
            if key == "send":
                with self.lock:
                    self.msg.data = command[key]
        return {}


Registry.register_resource_creator(
    Generic.SUBTYPE,
    RosTopicPublisher.MODEL,
    ResourceCreatorRegistration(RosTopicPublisher.new, RosTopicPublisher.validate_config)
)
