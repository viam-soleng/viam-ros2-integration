"""
configuration:
{
 cancel_on_reconfigure: true,
 hold_results: true,
 hold_last_n: 20,
 action_server_timeout: 10,
 actions: [
  {
    name: "action_name",
    type: "dot.path.to.datatype"
  }
 ]


TODO:
  1. update service:
     need more service methods for getting more data
  2. streaming results?
  3.
"""
import collections
import importlib
import json
import logging
import uuid

from datetime import datetime as dt
from threading import Lock
from typing import ClassVar, Deque, List, Mapping, Sequence, Union

from rclpy.action import ActionClient
from typing_extensions import Self
from viam.logging import getLogger
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ServiceConfig
from viam.resource.base import ResourceBase, ResourceName
from viam.resource.types import Model, ModelFamily
from viam.utils import struct_to_dict

from components.viam_ros_node import ViamRosNode
from proto.ros2_action_pb2 import ActionResponse

from .api import ROS2ActionService


class MyROS2ActionService(ROS2ActionService, Reconfigurable):

    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'ros2'), 'action_client')

    # Instance variables
    ros_node: Union[ViamRosNode, None]  # ros node that the action client will use
    logger: logging.Logger              # logger
    cancel_on_reconfigure: bool         # should service cancel existing actions on reconfigure (DEFAULT: False)
    hold_results: bool                  # should service hold previous results for reporting (DEFAULT: True)
    hold_last_n: int                    # how many results should we hold (DEFAULT: 10)
    action_server_timeout: int          # timeout (s) for waiting for action server (DEFAULT: 10)
    results: Union[Deque, None]         # results: {id: #, name: '', time: '', status: ''}
    actions: Union[List, None]          # configured actions {name: '', type: '', class: ''},
                                        # class is added during reconfigure
    current_actions: List               # current running actions: {name: '', start: '', goal_handler: '', future: ''}
                                        # goal_handler is a class
    lock: Lock

    def __init__(self, name: str):
        super().__init__(name)
        self.ros_node = None
        self.logger = getLogger(f'{__name__}.{self.__class__.__name__}')
        self.actions = None
        self.results = None
        self.current_actions = []
        self.cancel_on_reconfigure = False
        self.hold_last_n = 10
        self.action_server_timeout = 10
        self.lock = Lock()

    def _goal_done_callback(self, future) -> None:
        """

        """
        for action in self.current_actions:
            print(f'action: {action}')
            if action['future'] == future:
                with self.lock:
                    goal_handler = future.result()
                    action['goal_handler'] = goal_handler
                    if not goal_handler.accepted:
                        self.logger.error(f'goal {action["name"]}, was not accepted, removing from current actions')
                        self.results.append({
                            'id': uuid.uuid4(),
                            'name': action['name'],
                            'time': str(dt.now),
                            'status': f'goal was not accepted: {goal_handler.accepted}'
                        })
                        self.current_actions.remove(action)
                        return
                    self.logger.debug('goal was accepted and is executing, adding response callback')
                    action['goal_handler'].get_result_async()
                    action['goal_handler'].add_done_callback(self._goal_response_done_callback)
                    return
        self.logger.error('could not find action associated with future')

    def _goal_response_done_callback(self, future) -> None:
        """

        """
        for action in self.current_actions:
            if action['goal_handler'] == future:
                with self.lock:
                    self.logger.debug(f'found action: {action["name"]}, getting results and cleaning actions')
                    self.results.append({
                        'id': uuid.uuid4(),
                        'name': action['name'],
                        'time': str(dt.now()),
                        'status': future.result()
                    })
                    self.current_actions.remove(action)
        self.logger.error('could not find goal_handler associated with future')

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
        actions = config.attributes.fields['actions'].list_value
        if len(actions) == 0:
            raise Exception('actions attribute is required for this service to work.')
        return []

    # Handles attribute reconfiguration
    def reconfigure(
        self, config: ServiceConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        attributes_dict = struct_to_dict(config.attributes)

        self.actions = attributes_dict['actions']

        if 'hold_results' not in attributes_dict:
            self.logger.debug('setting hold_results to True (DEFAULT)')
            self.hold_results = True
        else:
            self.hold_results = attributes_dict['hold_results']

        if 'hold_last_n' not in attributes_dict:
            self.logger.debug('setting hold_last_n to 10 (DEFUALT)')
            self.hold_last_n = 10
        else:
            self.hold_last_n = int(attributes_dict['hold_last_n'])

        if 'action_server_timeout' not in attributes_dict:
            self.logger.debug('setting action_server_timeout to 10 seconds (DEFAULT)')
            self.action_server_timeout = 10
        else:
            self.action_server_timeout = int(attributes_dict['action_server_timeout'])

        if 'cancel_on_reconfigure' not in attributes_dict :
            self.logger.debug('setting cancel_on_reconfigure to False (DEFAULT)')
            self.cancel_on_reconfigure = False
        else:
            self.cancel_on_reconfigure = attributes_dict['cancel_on_reconfigure']

        self.results = collections.deque(maxlen=self.hold_last_n)
        # are there current actions running?
        if self.ros_node is not None:
            if len(self.current_actions) > 0 and self.cancel_on_reconfigure:
                for action in self.current_actions:
                    self.logger.info(f'stopping action {action["name"]} and removing from current actions')
                    # this will block (it is intended - but might need to revisit)
                    action['goal_handle'].cancel_goal()
                    self.current_actions.remove(action)
            else:
                self.current_actions.clear()
        else:
            self.ros_node = ViamRosNode.node

        for action in self.actions:
            self.logger.debug(f'reviewing action {action["name"]}')
            type_info = action['type'].rsplit('.', 1)
            try:
                lib = importlib.import_module(type_info[0])
                action['class'] = getattr(lib, type_info[1])
            except ModuleNotFoundError as mnfe:
                self.logger.error(f'problem importing module for {action["name"]}, type failed: {action["type"]}')
                self.logger.error(f'error: {mnfe}, removing action from list')
                self.actions.remove(action)

    async def send_goal(self, goal_name: str) -> ActionResponse:
        for action in self.actions:
            if action['name'] == goal_name:
                # attempt action
                action_client = ActionClient(self.ros_node, action['class'], action['name'])
                result = action_client.wait_for_server(timeout_sec=self.action_server_timeout)
                if not result:
                    self.logger.error(f'ROS action server timeout({self.action_server_timeout}) exceeded')
                    self.logger.error(f'cannot continue')
                    return ActionResponse(response=f'action server timeout: {goal_name} cannot continue')

                action_data = {
                    'name': action['name'],
                    'start': str(dt.now()),
                    'future': action_client.send_goal_async(action['class'].Goal())
                }
                action_data['future'].add_done_callback(self._goal_done_callback)

        return ActionResponse(response='action not found')

    async def cancel_goal(self, goal_name: str) -> ActionResponse:
        for action in self.current_actions:
            if action['name'] == goal_name and 'goal_handler' in action:
                with self.lock:
                    action['goal_handler'].cancel_goal_async()
                    action['goal_handler'].addadd_done_callback(self._goal_response_done_callback)
                    return ActionResponse(response=f'called cancel on {action["name"]}')

    async def goal_status(self, goal_name: str) -> ActionResponse:
        for result in self.results:
            if result['name'] == goal_name:
                return ActionResponse(response=result['status'])