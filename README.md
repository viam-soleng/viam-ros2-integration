# viam-ros2-module

This is a simple viam-ros integration which has been tested with ROS2 Humble turtlebot4 & turtlebot4-lite

## Overview

This project makes use of the ROS python api [rclpy](https://docs.ros2.org/foxy/api/rclpy/index.html) and [viam sdk](https://python.viam.dev/)
to wrap a core set of topics to read & write data to our ROS robot.

### Supported Conversions
This integration supports the ability to convert any message to a viam compatible message, for certain messages we 
convert to meaningful components to allow for richer Viam integrations.

1. Twist Messages to Viam Base Component: this supports sending move commands from viam and translating to ROS2 twist 
messages.
2. LaserScan to PointCloud for viam point cloud view and slam: this supports conversion to a point cloud format that Viam
can process and use in higher level services like SLAM which uses [cartographer](https://docs.viam.com/services/slam/cartographer/)
for building maps
3. IMU to movement sensor, for acceleration and other movement data
4. Any other type can currently be converted to a sensor message to display the ROS message, this allows all data (including
above conversions [1-3]) to be collected and sent to the viam cloud to allow users to manage data in near-real time.

### Installation

#### Install Viam
To install viam follow the instructions on our [docs](https://docs.viam.com/installation/)

#### Prepare DDS
If we are using fast DDS we need to first enable fast DDS to use UDP only, the [fastdd_rpi.xml](./sample_configs/fastdds_rpi.xml) 
is an example of what the configuration should look like.


#### Deploy modular resource
To install, we can choose this module from our [module registry](https://app.viam.com/registry) and install it, use
the [sample_config.json](./sample_configs/sample_config.json) to configure your robot.

#### Prepare ROS2 Environment
Every ROS2 environment potentially has a custom configuration, this includes namespaces, environment variables etc.
Once we selected the module we are deploying we need to add the `env` key to the ROS2 module json which contains a 
json document of `ENV: VAR` pairs.

##### Supported Variables

1. **ROS_ENV**: path to ros setup.bash, for example: `/opt/ros2/humble/setup.bash`
2. **OVERLAYS**: one or more custom overlays which hold our specific code, for example: `/path/to/ws1/setup.bash:/path/to/ws2/setup.bash`
3. **VIAM_NODE_NAME**: the name of the ros node to be created for the module, the default name is: `viam_node`
4. **VIAM_ROS_NAMESPACE**: the name of the ros namespace if one is used
5. **CACHE_DIR**: a path to a cache directory

#### Install Locally

When developing, follow the instructions here: [INSTALL.md](./INSTALL_LOCALLY)

### NOTE: Viam running as root and rmw_fastrtps_cpp
By default, rmw_fastrtps_cpp uses [eProsima](https://www.eprosima.com/index.php) DDS implementation, 
which makes use of [shared memory transports](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html)
when communication is between entities running on the same processing unit (or host).

Since viam by default starts as root, there are some shared memory issues. The current fix is to update
the DDS implementation to use `UDPv4` as the only transport, see [fastdds_rpi.xml](./sample_configs/fastdds_rpi.xml) for 
the example config used in turtlebot4.

For non-turtlebot4 implementations, update the specific fastdds_rpi.xml file in you implementation.

## Contact & Contributions
We welcome pull requests and issues, if there are any issues you can email us at:

* [shawn@viam.com](mailto:shawn@viam.com)
* [solution-eng@viam.com](mailto:solution-eng@viam.com)

# References
1. [viam](https://docs.viam.com)
2. [humble](https://docs.ros.org/en/humble/index.html)
3. [turtlebot4](https://clearpathrobotics.com/turtlebot-4/)
4. [turtlebot4 user guide](https://turtlebot.github.io/turtlebot4-user-manual/)

