import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from irobot_create_msgs.action import Dock, Undock


class DockClient(Node):
    def __init__(self):
        super().__init__('dock_action_node')
        self._undock_client = ActionClient(self, Undock, 'undock')
        self._dock_client = ActionClient(self, Dock, 'dock')
        self._future = None
        self._get_future_result = None

    def undock(self):
        goal_msg = Undock.Goal()
        self._undock_client.wait_for_server()
        self._future = self._undock_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self.goal_response_callback)
        return self._future

    def dock(self):
        goal_msg = Dock.Goal()
        self._dock_client.wait_for_server()
        self._future = self._dock_client.send_goal_async(goal_msg)
        self._future.add_done_callback(self.goal_response_callback)
        return self._future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('goal rejected')
            return
        self.get_logger().info('goal accepted')
        self._get_future_result = goal_handle.get_result_async()
        self._get_future_result.add_done_callback(self.callback_done)

    def callback_done(self, future):
        result = future.result()
        dir(result)
        self.get_logger().info(f'result: {result}')


def main(args=None):
    rclpy.init(args=args)
    action_client = DockClient()
    action_client.undock()
    action_client.dock()
    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
