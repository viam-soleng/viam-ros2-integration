#!/bin/bash

#
# this file is used to build out our virtual environment
# and add any environment variables needed to our run.sh
# script
#
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#
# export underlay & add overlays as needed
#
. /etc/viam/setup.bash

#
# export MODULE_NAME
#
export MODULE_NAME="viam-ros2-module"
export MODULE_VERSION=0.0.1
export SHOULD_INSTALL="FALSE"

if [[ -f /var/viam/${MODULE_NAME}/VERSION ]]; then
  # did version change
  CURRENT_VERSION=`cat /var/viam/${MODULE_NAME}/VERSION`
  if [[ ${CURRENT_VERSION} != ${MODULE_VERION} ]]; then
    SHOULD_INSTALL="TRUE"
  fi
else
  mkdir -p /var/viam/${MODULE_NAME} 2&>1 /dev/null
  [[ $? -ne 0 ]] && echo "MUST FAIL HERE"
  echo ${MODULE_VERSION} >> /var/viam/${MODULE_NAME}/VERSION
  SHOULD_INSTALL="TRUE"
fi

#
# setup of virtual environment
# TODO: create flag vs. update flag
#
if [[ ! -d ${SCRIPT_DIR}/venv/bin/python ]]; then
  echo "Setting up virtual environment & installing requirements"
  python3 -m venv ${SCRIPT_DIR}/venv
  exec ${SCRIPT_DIR}/venv/bin/python -m pip install -r ${SCRIPT_DIR}/requirements.txt
else
  echo "virtual environment exists, will not run setup"
fi


# setup LD_LIBRARY_PATH for viam
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib/python3.10/dist-packages/viam/rpc/:${SCRIPT_DIR}/venv/lib/python3.10/site-packages/viam/rpc/

# TODO: ctrl-c seems to kill the run.sh script while leaving the child process running
exec ${SCRIPT_DIR}/venv/bin/python3 ${SCRIPT_DIR}/src/main.py $@
