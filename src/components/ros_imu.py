"""
ros_imu converts the ROS Imu message to a Viam movement sensor configuration
A ros_imu only provides support for:
1. linear acceleration
2. angular velocity
3. orientation

the movementsensor for viam also provides support for:
1. position
2. compass heading
3. linear velocity

If we need support for other types we can use this interface to create GPS, compass, and
other implementations as needed for ROS
"""
import rclpy
import viam
from threading import Lock
from utils import quaternion_to_orientation
from typing import Any, ClassVar, Dict, Mapping, Optional, Sequence, Tuple
from typing_extensions import Self
from viam.components.movement_sensor import MovementSensor, Orientation, Vector3
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import Imu
from .viam_ros_node import ViamRosNode


class Unimplemented(Exception):
    """
    simple exception
    TODO: this will go away with movement sensor updates (coming in RDK)
    """

    def __init__(self):
        super().__init__()


class RosImuProperties(MovementSensor.Properties):
    """
    Viam can support six different types of movements in one, while the
    ROS IMU message only supports a few

    By setting this property we can programmatically ask Viam what options
    are supported
    """

    def __init__(self):
        super().__init__(
            linear_acceleration_supported=True,
            angular_velocity_supported=True,
            orientation_supported=True,
            position_supported=False,
            compass_heading_supported=False,
            linear_velocity_supported=False
        )


class RosImu(MovementSensor, Reconfigurable):
    """
    RosImu converts ROS IMU message to movementsensor values to be used
    by the viam RDK
    """
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'ros2'), 'imu')
    ros_topic: str
    ros_node: Node
    subscription: Subscription
    msg: Imu
    lock: Lock
    props: RosImuProperties

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        """
        new() creates a new RosImu object to be used by the viam RDK
        """
        imu = cls(config.name)
        imu.ros_node = None
        imu.props = RosImuProperties()
        imu.reconfigure(config, dependencies)
        return imu

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """
        validate_config ensures that the ros_topic is set
        """
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        """
        reconfigure will configure the ROS node as needed during startup and when the
        RDK configuration is updated.
        """
        self.ros_topic = config.attributes.fields['ros_topic'].string_value

        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.ros_node.create_subscription(
            Imu,
            self.ros_topic,
            self.subscriber_callback,
            qos_profile=qos_policy
        )
        self.lock = Lock()
        self.msg = None

    def subscriber_callback(self, msg: Imu) -> None:
        """
        Update the internal message with the data which comes off the topic
        """
        with self.lock:
            self.msg = msg

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[viam.components.movement_sensor.GeoPoint, float]:
        """
        get_position is not supported by the ROS IMU topic
        """
        raise Unimplemented()

    async def get_linear_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        """
        get_linear_velocity is not supported by the ROS Imu message
        """
        raise Unimplemented()

    async def get_angular_velocity(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        """
        get_angular_velocity returns the angular velocity vector from the IMU message
        """
        if self.msg is None:
            raise Exception("ros imu message not ready")
        av = self.msg.angular_velocity
        return Vector3(x=av.x, y=av.y, z=av.z)

    async def get_linear_acceleration(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Vector3:
        """
        get_linear_acceleration returns the linear acceleration vector from the IMU message
        """
        if self.msg is None:
            raise Exception("ros imu message not ready")
        la = self.msg.linear_acceleration
        return Vector3(x=la.x, y=la.y, z=la.z)

    async def get_compass_heading(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        """
        get_compass_heading is not supported by the ROS IMU message
        """
        raise Unimplemented()

    async def get_orientation(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> viam.components.movement_sensor.Orientation:
        """
        get_orientation returns the orientation represented as an orientation vector from the
        Imu orientation quaternion
        """
        if self.msg is None:
            raise Exception("ros imu message not ready")
        o = self.msg.orientation
        return quaternion_to_orientation(o.w, o.x, o.y, o.z)

    async def get_accuracy(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, float]:
        """
        get_accuracy is not implemented
        """
        raise Unimplemented()

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> MovementSensor.Properties:
        """
        get_properties returns the supported commands for the imu
        """
        return self.props

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """
        do_command - not implemented
        """
        raise Unimplemented()


"""
Register the new MODEL as well as define how the object is validated 
and created
"""
Registry.register_resource_creator(
    MovementSensor.SUBTYPE,
    RosImu.MODEL,
    ResourceCreatorRegistration(RosImu.new, RosImu.validate_config)
)
