import abc
from typing import Final

from grpclib.client import Channel
from grpclib.server import Stream

from viam.resource.rpc_service_base import ResourceRPCServiceBase
from viam.resource.types import RESOURCE_TYPE_SERVICE, API
from viam.services.service_base import ServiceBase

from proto.ros2_service_grpc import ROS2ServiceServiceBase, ROS2ServiceServiceStub
from proto.ros2_service_pb2 import ServiceRequest, ServiceResponse


class ROS2ServiceService(ServiceBase):
    """
    Viam ROS 2 logger service subclass of the ServiceBase class including additional abstract methods to be implemented
    """

    API: Final = API("viam-soleng", RESOURCE_TYPE_SERVICE, "service_client")

    @abc.abstractmethod
    async def call(self, goal_name: str) -> ServiceResponse: ...

    @abc.abstractmethod
    async def destroy(self, goal_name: str) -> ServiceResponse: ...

    @abc.abstractmethod
    async def status(self, goal_name: str) -> ServiceResponse: ...


class ROS2ServiceRPCService(ROS2ServiceServiceBase, ResourceRPCServiceBase):
    RESOURCE_TYPE = ROS2ServiceService

    async def send_message(
        self, stream: Stream[ServiceRequest, ServiceResponse]
    ) -> None:
        request = await stream.recv_message()
        assert request is not None
        name = request.name
        service = self.get_resource(name)
        resp = await service.status()
        await stream.send_message(resp)

    async def Call(self, stream: Stream[ServiceRequest, ServiceResponse]) -> None:
        await self.send_message(stream)

    async def Destroy(self, stream: Stream[ServiceRequest, ServiceResponse]) -> None:
        await self.send_message(stream)

    async def Status(self, stream: Stream[ServiceRequest, ServiceResponse]) -> None:
        await self.send_message(stream)


class ROS2ServiceClient(ROS2ServiceService):
    def __init__(self, name: str, channel: Channel) -> None:
        self.channel = channel
        self.client = ROS2ServiceServiceStub(channel)
        super().__init__(name)

    async def call(self, goal_name: str) -> str:
        resp: ServiceResponse = await self.client.Call(
            ServiceRequest(name=self.name, service=goal_name)
        )
        return resp.response

    async def destroy(self, goal_name: str) -> str:
        resp: ServiceResponse = await self.client.Destroy(
            ServiceRequest(name=self.name, service=goal_name)
        )
        return resp.response

    async def status(self, goal_name: str) -> str:
        resp: ServiceResponse = await self.client.Status(
            ServiceRequest(name=self.name, service=goal_name)
        )
        return resp.response
