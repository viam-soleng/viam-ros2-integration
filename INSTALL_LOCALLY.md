# Installing ROS Module Locally

For easier development and/or debugging you can directly clone this repository onto your machine and add it as local module in your Viam configuration.

## Prerequisits

It is assumed you have already successfully deployed the Viam Server and it has successfully connected to the Viam Cloud platform as explained here: [instructions](https://docs.viam.com/installation/)

## 1. Clone Repository

Clone this repository onto your machine.

## 2. Configure Local Module

To add a local module, all you have to do is to go into your Viam machine configuration, click on the `+` sign, choose `Local Module` and again `Local Module`.
then add the path to the run.sh file on your system into into `Executable Path` and save the configuration.

You can also add the configuration directly in plain JSON mode:

```json
  "modules": [
    {
      "type": "local",
      "name": "viam-ros2-local",
      "executable_path": "e.g. /home/user/viam-ros2-integration/run.sh"
    }
  ]
```

Keep in mind that you likely have to add environment variables as explained here: [ROS2 Integration Configuration](README.md#ros2-integration-configuration).

# Future Improvements

1. module config: jira has been filed, this will support:
   1. running as non-root user
   2. setting up environment variables (Implemented)
   3. remove dds update requirements
2. expanded support for services and actions
3. code deploy and best practices

## Contact & Contributions

We welcome pull requests and issues, if there are any issues you can email us at:

- [solution-eng@viam.com](mailto:solution-eng@viam.com)

[^1]: jira tickets have been filed for improvements to the module to support non-root users
