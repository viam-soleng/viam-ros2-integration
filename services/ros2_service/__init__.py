from viam.resource.registry import (
    Registry,
    ResourceCreatorRegistration,
    ResourceRegistration,
)

from .api import ROS2ServiceClient, ROS2ServiceRPCService, ROS2ServiceService
from .ros2_service import MyROS2ServiceService

Registry.register_api(
    ResourceRegistration(
        ROS2ServiceService,
        ROS2ServiceRPCService,
        lambda name, channel: ROS2ServiceClient(name, channel),
    )
)

Registry.register_resource_creator(
    ROS2ServiceService.API,
    MyROS2ServiceService.MODEL,
    ResourceCreatorRegistration(MyROS2ServiceService.new),
)
