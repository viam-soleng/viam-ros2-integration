#!/bin/bash

#
# this file is used to build out our virtual environment
# and add any environment variables needed to our run.sh
# script
#
# TODO: Organize what an install will look like it must include:
#       1. Ability to be reused
#       2. can we use github actions to incorporate from other repo
#
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#
# export which should contain ROS env (see INSTALL.md)
#
. /etc/viam/setup.bash

#
# export MODULE_NAME
#
export MODULE_NAME="viam-ros2-module"

#
# setup of virtual environment
# TODO: create flag vs. update flag
#
if [[ ! -f ${SCRIPT_DIR}/venv/bin/python ]]; then
  echo "Setting up virtual environment & installing requirements"
  python3 -m venv ${SCRIPT_DIR}/venv --system-site-packages
  ${SCRIPT_DIR}/venv/bin/python -m pip install -r ${SCRIPT_DIR}/requirements.txt
else
  echo "virtual environment exists, will not run setup"
fi


# setup LD_LIBRARY_PATH for viam rpc code (needed by RosImu)
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${SCRIPT_DIR}/venv/lib/python3.10/site-packages/viam/rpc

printenv

# TODO: ctrl-c seems to kill the run.sh script while leaving the child process running
exec "${SCRIPT_DIR}"/venv/bin/python3 "${SCRIPT_DIR}"/src/main.py "$@"
