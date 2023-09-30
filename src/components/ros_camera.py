"""
ros_camera

converts ros image to viam image which allows for higher level processing by viam

TODO: should we expose the intrinsic and distortion parameters
      should we raise an exception rather than return an empty image
"""
from PIL import Image
import rclpy
import viam
from threading import Lock
from typing import ClassVar, Mapping, Optional, Sequence, Tuple, List
from typing_extensions import Self
from viam.components.camera import Camera, IntrinsicParameters, DistortionParameters
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes
from rclpy.node import Node
from rclpy.subscription import Subscription
from sensor_msgs.msg import Image as ROSImage
from .viam_ros_node import ViamRosNode
from cv_bridge import CvBridge


class RosCamera(Camera, Reconfigurable):
    """
    RosCamera converts ROS Image message to Viam Image
    """
    MODEL: ClassVar[Model] = Model(ModelFamily("viamlabs", "ros2"), "camera")

    # Instance variables
    ros_topic: str
    ros_node: Node
    subscription: Subscription
    props: Camera.Properties
    lock: Lock
    image: ROSImage
    bridge: CvBridge

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """
        new() creates new instance of a RosCamera
        """
        camera = cls(config.name)
        camera.ros_node = None
        camera.props = Camera.Properties(
            supports_pcd=False,
            distortion_parameters=DistortionParameters(),
            intrinsic_parameters=IntrinsicParameters(),
        )
        camera.reconfigure(config, dependencies)
        return camera

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """
        validate_config is used to validate the viam configuration of the RosCamera object, the
        only attribute required is "ros_topic"
        """
        topic = config.attributes.fields['ros_topic'].string_value
        if topic == '':
            raise Exception('ros_topic required')
        return []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> None:
        """
        reconfigure is called when new is called as well as when the RDK configuration is
        updated, during this process we will update the ros node with the new configuration
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
            depth=1,
        )

        self.subscription = self.ros_node.create_subscription(
            ROSImage, self.ros_topic, self.subscriber_callback, qos_profile=qos_policy
        )
        self.bridge = CvBridge()
        self.lock = Lock()

    def subscriber_callback(self, image: ROSImage) -> None:
        """
        subscriber_callback called when we get an image off the ROS topic
        """
        with self.lock:
            self.image = image

    async def get_image(
        self,
        mime_type: str = '',
        timeout: Optional[float] = None,
        **kwargs
    ) -> Image:
        """
        get_image will either return an empty image if the topic is not ready
        or a new PIL.Image from the ROS Image message
        """
        img = self.image
        if img is None:
            return Image.new(mode='RGB', size=(250, 250))
        else:
            return Image.fromarray(self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough'))

    async def get_images(
        self,
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[List[viam.media.video.NamedImage], viam.proto.common.ResponseMetadata]:
        """
        get_images currently not supported
        """
        raise NotImplementedError()

    async def get_point_cloud(
        self,
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[bytes, str]:
        """
        get_point_cloud is not supported for the basic ROS Image
        """
        raise NotImplementedError()

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        """
        get_properties returns the properties supported by the camera
        """
        return self.props

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs,
    ) -> Mapping[str, ValueTypes]:
        """
        do_command is currently not implemented
        """
        raise NotImplementedError()


"""
Register the new MODEL as well as define how the object is validated 
and created
"""
Registry.register_resource_creator(
    Camera.SUBTYPE,
    RosCamera.MODEL,
    ResourceCreatorRegistration(RosCamera.new, RosCamera.validate_config),
)
