"""
ros_lidar.py

ROS produces LaserScan messages for lidar data, this data must be
transformed to point cloud data to be consumed by the RDK

Currently, we support 2D scans without intensity
"""
import math
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.subscription import Subscription
from PIL.Image import Image
from sensor_msgs.msg import LaserScan
from threading import Lock
from typing import ClassVar, List, Mapping, Optional, Sequence, Tuple, Union
from typing_extensions import Self
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters, RawImage
from viam.media.video import NamedImage
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from .viam_ros_node import ViamRosNode
from viam.media.video import CameraMimeType

# PCD required header information
VERSION = 'VERSION .7\n'
FIELDS = 'FIELDS x y z\n'
SIZE = 'SIZE 4 4 4\n'
TYPE_OF = 'TYPE F F F\n'
COUNT = 'COUNT 1 1 1\n'
HEIGHT = 'HEIGHT 1\n'
VIEWPOINT = 'VIEWPOINT 0 0 0 1 0 0 0\n'
DATA = 'DATA binary\n'


class RosLidar(Camera, Reconfigurable):
    """
    RosLidar supports consuming laserscan messages and converting to the
    RDK Pointcloud type.

    For this integration we require the ros topic to consume
    """
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'ros2'), 'lidar')
    ros_topic: str
    ros_node: Node
    subscription: Subscription
    ros_lidar_properties: Camera.Properties
    lock: Lock
    msg: LaserScan

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        """
        new creates a new lidar object which will subscribe to the configured lidar scan topic
        """
        lidar = cls(config.name)
        lidar.ros_node = None
        lidar.reconfigure(config, dependencies)
        return lidar

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """
        validate_config requires one component attribute which is the ros_topic to
        subscribe to
        """
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> None:
        """
        reconfigure is called when the RDK configuration is changed, during this process
        we will update the ros node to ensure this component is properly configured base
        on any system configuration changes
        """
        self.ros_topic = config.attributes.fields['ros_topic'].string_value
        self.ros_lidar_properties = Camera.Properties(
            supports_pcd=True,
            intrinsic_parameters=IntrinsicParameters(
                width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0
            ),
            distortion_parameters=DistortionParameters(model='')
        )
        
        if self.ros_node is not None:
            if self.subscription is not None:
                self.ros_node.destroy_subscription(self.subscription)
        else:
            self.ros_node = ViamRosNode.get_viam_ros_node()

        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.SYSTEM_DEFAULT,
            history=rclpy.qos.HistoryPolicy.SYSTEM_DEFAULT,
            depth=1
        )
        self.subscription = self.ros_node.create_subscription(
            LaserScan,
            self.ros_topic,
            self.subscriber_callback,
            qos_profile=qos_policy
        )
        self.lock = Lock()
        self.msg = None

    def subscriber_callback(self, msg: LaserScan) -> None:
        """
        subscriber_callback to listen for laser scan messages
        """
        self.msg = msg

    async def get_image(self, mime_type: str='', *, timeout: Optional[float]=None, **kwargs) -> Union[Image, RawImage]:
        """
        get_image
        in Viam a lidar is considered a type of camera meaning it must implement all the
        camera interfaces, since a generic lidar does not have the concept of images we
        will raise the unimplemented error
        """
        raise NotImplementedError()

    async def get_images(
            self,
            *,
            timeout: Optional[float]=None,
            **kwargs
    ) -> Tuple[List[NamedImage], ResponseMetadata]:
        """
        get_images
        another camera interface which is needed for cameras, in this case we raise the
        unimplemented error
        """
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[bytes, str]:
        """
        get_point_cloud
        In viam lidar points are represented by point cloud elements,
        """
        with self.lock:
            msg = self.msg

        if msg is None:
            raise Exception('laserscan msg not ready')

        pdata = []
        for i, r in enumerate(msg.ranges):
            if r < msg.range_min or r > msg.range_max:
                continue

            ang = msg.angle_min + (float(i) * msg.angle_increment)
            x = math.cos(ang) * r
            y = math.sin(ang) * r
            pdata.append(x)
            pdata.append(y)
            pdata.append(float(0))

        width = f'WIDTH {len(pdata)}\n'
        points = f'POINTS {len(pdata)}\n'
        header = f'{VERSION}{FIELDS}{SIZE}{TYPE_OF}{COUNT}{width}{HEIGHT}{VIEWPOINT}{points}{DATA}'
        a = np.array(pdata, dtype='f')
        h = bytes(header, 'UTF-8')

        return h + a.tobytes(), CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        """
        Return the base lidar properties
        """
        return self.ros_lidar_properties


"""
Register the new MODEL as well as define how the object is validated 
and created
"""
Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosLidar.MODEL,
    ResourceCreatorRegistration(RosLidar.new, RosLidar.validate_config)
)

