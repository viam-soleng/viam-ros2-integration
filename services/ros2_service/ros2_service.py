import logging
from threading import Lock
from typing import ClassVar, Mapping, Sequence, Union

from typing_extensions import Self
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ServiceConfig
from viam.resource.base import ResourceBase, ResourceName
from viam.resource.types import Model, ModelFamily

from components.viam_ros_node import ViamRosNode
from proto.ros2_service_pb2 import ServiceResponse

from .api import ROS2ServiceService


class MyROS2ServiceService(ROS2ServiceService, Reconfigurable):

    MODEL: ClassVar[Model] = Model(ModelFamily("viam-soleng", "ros2"), "ros2_service")

    # Instance variables
    ros_node: Union[ViamRosNode, None]
    lock: Lock
    logger: logging.Logger

    def __init__(self, name: str):
        super().__init__(name)
        self.ros_node = None
        self.lock = Lock()
        self.logger = getLogger(f'{__name__}.{self.__class__.__name__}')

    # Constructor
    @classmethod
    def new(
        cls, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        service = cls(config.name)
        service.reconfigure(config, dependencies)
        return service

    # Validates JSON Configuration
    @classmethod
    def validate_config(cls, config: ServiceConfig) -> Sequence[str]:
        return []

    # Handles attribute reconfiguration
    def reconfigure(
        self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        if self.ros_node is None:
            self.ros_node = ViamRosNode.node
        pass

    async def call(self, goal_name: str) -> ServiceResponse:
        pass

    async def destroy(self, goal_name: str) -> ServiceResponse:
        pass

    async def status(self, goal_name: str) -> ServiceResponse:
        pass


