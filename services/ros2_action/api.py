import abc
from typing import Final

from grpclib.client import Channel
from grpclib.server import Stream

from viam.resource.rpc_service_base import ResourceRPCServiceBase
from viam.resource.types import RESOURCE_TYPE_SERVICE, API
from viam.services.service_base import ServiceBase

from proto.ros2_action_grpc import ROS2ActionServiceBase, ROS2ActionServiceStub
from proto.ros2_action_pb2 import ActionRequest, ActionResponse


class ROS2ActionService(ServiceBase):
    """
    Viam ROS 2 logger service subclass of the ServiceBase class including additional abstract methods to be implemented
    """

    API: Final = API("viam-soleng", RESOURCE_TYPE_SERVICE, "action_client")

    @abc.abstractmethod
    async def send_goal(self, goal_name: str) -> ActionResponse: ...

    @abc.abstractmethod
    async def cancel_goal(self, goal_name: str) -> ActionResponse: ...

    @abc.abstractmethod
    async def goal_status(self, goal_name: str) -> ActionResponse: ...


class ROS2ActionRPCService(ROS2ActionServiceBase, ResourceRPCServiceBase):
    RESOURCE_TYPE = ROS2ActionService

    async def SendGoal(self, stream: Stream[ActionRequest, ActionResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.send_goal(goal_name=request.action)
        await stream.send_message(resp)

    async def CancelGoal(self, stream: Stream[ActionRequest, ActionResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.cancel_goal(goal_name=request.action)
        await stream.send_message(resp)

    async def GoalStatus(self, stream: Stream[ActionRequest, ActionResponse]) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.goal_status(goal_name=request.action)
        await stream.send_message(resp)


class ROS2ActionClient(ROS2ActionService):
    def __init__(self, name: str, channel: Channel) -> None:
        self.channel = channel
        self.client = ROS2ActionServiceStub(channel)
        super().__init__(name)

    async def goal_status(self, goal_name: str) -> str:
        resp: ActionResponse = await self.client.GoalStatus(
            ActionRequest(name=self.name, action=goal_name)
        )
        return resp.response

    async def cancel_goal(self, goal_name: str) -> str:
        resp: ActionResponse = await self.client.CancelGoal(
            ActionRequest(name=self.name, action=goal_name)
        )
        return resp.response

    async def send_goal(self, goal_name: str) -> str:
        resp: ActionResponse = await self.client.SendGoal(
            ActionRequest(name=self.name, action=goal_name)
        )
        return resp.response
