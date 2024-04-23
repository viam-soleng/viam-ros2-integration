# Building

Since this a Python application very little is needed to set up the environment. We have included the base set of 
requirements for the Viam libraries with the assumption that ROS2 python packages are installed and sourced 
from the ROS2 environment scripts.

To get started, fork the repository to your github account.

As you develop you can create a branch based on the improvement or fix you are introducing and create a pull
request as needed.

Over time our process might change as needed.

# Python
If additional libraries are added to the environment, to update the requirements.txt please use the following
command:
```shell
# this will only update the local virtual environment requirements
pip freeze -l > ./requirements.txt
```

# Proto generation
We make use of the [buf cli](https://buf.build/docs/installation), make sure
this is installed as well as the python plugins:

```shell
pip install "grpclib[protobuf]"
pip install mypy-protobuf

buf mod update
for i in `ls proto/*proto`; do buf generate $i; done
```

Note that if you install the viam-sdk it will install googleapis-common-protos
which will cause conflicts with proto generation.

## Contributions
We welcome pull requests and issues, if there are any issues you can email us at:

* [shawn@viam.com](mailto:shawn@viam.com)
* [solution-eng@viam.com](mailto:solution-eng@viam.com)