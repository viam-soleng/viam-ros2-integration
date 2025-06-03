from viam.resource.registry import (
    Registry,
    ResourceCreatorRegistration,
    ResourceRegistration,
)

from .api import ROS2ActionClient, ROS2ActionRPCService, ROS2ActionService
from .ros2_action import MyROS2ActionService

Registry.register_api(
    ResourceRegistration(
        ROS2ActionService,
        ROS2ActionRPCService,
        lambda name, channel: ROS2ActionClient(name, channel),
    )
)

Registry.register_resource_creator(
    ROS2ActionService.API,
    MyROS2ActionService.MODEL,
    ResourceCreatorRegistration(MyROS2ActionService.new),
)
