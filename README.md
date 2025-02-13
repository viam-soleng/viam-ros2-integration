# viam-ros2-module

This Viam - ROS2 integration allows you to connect to your ROS2 environment, publish messages, collect data and make it accessible through the Viam API layer as well as storing it locally on the system or upload it into the Viam cloud platform.
The integration has been tested with ROS2 Humble and Turtlebot 4.

This module is fully open source and contributions are very welcome!

## Overview

This project makes use of the ROS python api [rclpy](https://docs.ros2.org/foxy/api/rclpy/index.html) and [viam sdk](https://python.viam.dev/)
to wrap a core set of topics to read & write data to your ROS2 robot.

### Supported Conversions

This integration supports the ability to convert any message to a viam compatible message, for certain messages we
map them to Viam components which allow for richer Viam integrations.

1. Twist Messages to Viam Base Component: this supports sending move commands from viam and translating to ROS2 twist
   messages.
2. LaserScan to PointCloud for viam point cloud view and slam: this supports conversion to a point cloud format that Viam
   can process and use in higher level services like SLAM which uses [cartographer](https://docs.viam.com/services/slam/cartographer/)
   for building maps
3. IMU to movement sensor, for acceleration and other movement data
4. Any other type can currently be converted to a sensor message to display the ROS message, this allows all data (including
   above conversions [1-3]) to be collected and sent to the viam cloud to allow users to manage data in near-real time.

### Installation

#### Install Viam Server

To install viam follow the instructions on our [docs](https://docs.viam.com/installation/)

#### Deploy Viam ROS2 Integration Module

The easiest way to install this module is via our [Viam Registry](https://app.viam.com/registry). With regards to configuration you can have a look at this [sample_config.json](./sample_configs/sample_config.json). Makes sure you change values to meet your system requirements.

#### Viam Node UDP Only Mode

If Viam is deplody on the same system, due to the DDS default configuration shared memory "SHM" is used. The Viam node currently does not support shared memory and therefore "SHM" needs to be disabled so it uses UDP only. 

To do this, you have to copy/create a file containing this configration [fastdd_rpi.xml](./sample_configs/fastdds_rpi.xml) into a suitable path on your machine.

You can then load this configuration by adding the `FASTRTPS_DEFAULT_PROFILES_FILE` environment variable with the correct path for your system into your module configuration:

```json
  "modules": [
    {
      "type": "registry",
      "name": "viam-soleng_viam-ros2-integration",
      "module_id": "viam-soleng:viam-ros2-integration",
      "version": "0.0.11",
      "env": {
        "FASTRTPS_DEFAULT_PROFILES_FILE": "/opt/ros/humble/fastdds_rpi.xml" # Change accordingly
      }
    }
  ]
```

Additional information regarding [Shared Memory "SHM"](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html)

#### ROS2 Integration Configuration

Every ROS2 environment potentially has a custom configuration, this includes namespaces, environment variables etc.
Once we have installed the ROS2 module, we therefore likely have to add some additional environment variables.

##### Viam Specific Environment Variables

| Name                 | Inclusion | Description                                                                                                       |
| -------------------- | --------- | ----------------------------------------------------------------------------------------------------------------- |
| `ROS_ENV`            | Required  | Path to ros setup.bash, for example: `/opt/ros2/humble/setup.bash`                                                |
| `OVERLAYS`           | Optional  | One or more custom overlays which hold our specific code, e.g.: `/path/to/ws1/setup.bash:/path/to/ws2/setup.bash` |
| `VIAM_NODE_NAME`     | Optional  | The name of Viam ros node, the default name is: `viam_node`                                                       |
| `VIAM_ROS_NAMESPACE` | Optional  | The name of the ros namespace if one is used                                                                      |
| `CACHE_DIR`          | Optional  | A path to a cache directory                                                                                       |

You can set these environment variables as follows:

```json
  "modules": [
    {
      "type": "registry",
      "name": "viam-soleng_viam-ros2-integration",
      "module_id": "viam-soleng:viam-ros2-integration",
      "version": "0.0.11",
      "env": {
        "VariableName": "VALUE",
        ... : ... # Add additional variables here
      }
    }
  ]
```

#### Install Locally (Development)

For developing or testing the raw module can also be deployed locally. 

To do so follow the instructions here: [INSTALL.md](./INSTALL_LOCALLY.md)

## Troubleshooting

1. The ros2-integration module uses python virtual environments. Verify that the following package is installed:

```bash
sudo apt-get install -y python3-venv
```

2. The ros2-integration module also requires the python cv_bridge package which can be installed using the following command:

```bash
sudo apt-get install ros-<ROS Version Name>-cv-bridge
```

## Contact & Contributions

We welcome pull requests and issues, if there are any issues you can email us at:

- [solution-eng@viam.com](mailto:solution-eng@viam.com)

# References

1. [viam](https://docs.viam.com)
2. [humble](https://docs.ros.org/en/humble/index.html)
3. [turtlebot4](https://clearpathrobotics.com/turtlebot-4/)
4. [turtlebot4 user guide](https://turtlebot.github.io/turtlebot4-user-manual/)
