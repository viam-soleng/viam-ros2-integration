import asyncio
import os
import time
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.camera import Camera
from viam.components.base import Base
from viam.components.movement_sensor import MovementSensor
from viam.components.sensor import Sensor
from viam.services.vision import VisionClient
from services.ros2_action.api import ROS2ActionService


async def connect():

    opts = RobotClient.Options.with_api_key(
        api_key=os.environ['API_KEY'],
        api_key_id=os.environ['API_KEY_ID']
    )
    return await RobotClient.at_address(os.environ['VIAM_MACHINE'], opts)


async def main():
    machine = await connect()

    print('Resources:')
    print(machine.resource_names)

    # hazard_detection
    hazard_detection = Sensor.from_robot(machine, "hazard_detection")
    hazard_detection_return_value = await hazard_detection.get_readings()
    print(f"hazard_detection get_readings return value: {hazard_detection_return_value}")

    # battery_state
    battery_state = Sensor.from_robot(machine, "battery_state")
    battery_state_return_value = await battery_state.get_readings()
    print(f"battery_state get_readings return value: {battery_state_return_value}")

    # dock_status
    dock_status = Sensor.from_robot(machine, "dock_status")
    dock_status_return_value = await dock_status.get_readings()
    print(f"dock_status get_readings return value: {dock_status_return_value}")

    actionClient = ROS2ActionService.from_robot(machine, 'ros2action')
    print(f'actionClient: {actionClient}')
    await actionClient.send_goal('dock')
    time.sleep(45)

    # Don't forget to close the machine when you're done!
    await machine.close()


if __name__ == '__main__':
    asyncio.run(main())
