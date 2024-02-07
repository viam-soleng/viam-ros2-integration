# Installing ROS Module Locally

We currently support two ways of deploying the ROS2 module to the Viam process:
1. Through the [Viam Registry](https://app.viam.com/registry): more information can be found [here](https://docs.viam.com/registry/)
2. Build & Install locally: Typically used during development



These configurations have been tested on turtlebot4 machines, where we have had multiple systems
in one office which required testing with namespaces as well as domain ids.

## Requirements
1. viam account 
2. viam organization 
3. viam location
4. ROS2 robot
5. python venv module installed

## Viam Install
After creating you account you can follow these [instructions](https://docs.viam.com/installation/) to 
install Viam on your ROS2 robot.

## On-Robot Configuration

With our current release of the module registry there is a few things to do to prepare the environment 
to ensure viam can successfully connect with your ROS2 deployment, these instructions will change shortly.

### Preparing ROS2

With ROS2 and the use of DDS, we might need to configure our DDS instance to ensure Viam can successfully
connect to the ROS2 system. When using fastdds we might be using shared memory as the transport, if this 
is the case the Viam module will not be able to connect to the ROS2 node by default.

Currently, modules start as `root` by default[^1] which causes issue with shared memory as ROS2 typically
runs as a non-root user.  To compensate for this we have provided a [sample config](./sample_configs/fastdds_rpi.xml)
which shows how to configure the `UDP4` transport.

Based on the configuration you are using replace the contents of the xml file for your ROS2 DDS configuration
with the ones found in the link above.

If we are using docker, we should not require any changes to our system.

### Configuration



# Future Improvements
1. module config: jira has been filed, this will support:
   1. running as non-root user
   2. setting up environment variables
   3. remove dds update requirements
2. expanded support for services and actions
3. code deploy and best practices

## Contact & Contributions
We welcome pull requests and issues, if there are any issues you can email us at:

* [shawn@viam.com](mailto:shawn@viam.com)
* [solution-eng@viam.com](mailto:solution-eng@viam.com)

[^1]: jira tickets have been filed for improvements to the module to support non-root users