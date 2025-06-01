# TODO

## Overall

1. create test cases for each message type supported
2. improve documentation
3. more message conversion
4. improve ros_base (see note below)
5. set parameter config
6. Expose more of the RCLPY node (services, parameters, etc)
7. ros2_config.py: installation process
8. support calling services with action clients, etc.
9. makefile for building proto services
10. ros_camera.py to implement a generic is camera running check?

### ros_base
currently we set the ros_base to always publish even after we are sending 0's (stopped). 

We would like to only publish stop messages once (or a couple of times - UDP!) and then stop, so we can
then configure other actions etc. There will be testing on this.

## Contributions
We welcome pull requests and issues, if there are any issues you can email us at:

* [shawn@viam.com](mailto:shawn@viam.com)
* [solution-eng@viam.com](mailto:solution-eng@viam.com)