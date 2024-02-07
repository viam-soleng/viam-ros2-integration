#!/bin/bash

#
# this file is used to build out our virtual environment
# and add any environment variables needed to our run.sh
# script
#
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

#
# setup of virtual environment if it does not already exist
#
if [[ ! -f ${SCRIPT_DIR}/venv/bin/python ]]; then
  echo "[INFO] Attempting to setup python environment"
  python3 -m venv ${SCRIPT_DIR}/venv --system-site-packages &> /dev/null
  if [[ $? -ne 0 ]]; then
    echo "[ERROR] problem setting up virtual environment cannot continue (please review error logs)"
    exit 1
  fi
  "${SCRIPT_DIR}"/venv/bin/python -m pip install -r "${SCRIPT_DIR}"/requirements.txt &> /dev/null
  if [[ $? -ne 0 ]]; then
    echo "[ERROR] problem installing requirements, cannot continue (please review error logs)"
    exit 1
  fi
fi

# must be set in module
# shellcheck source=/dev/null
if [[ "${ROS_ENV}" ]]; then
  source "${ROS_ENV}"
else
  echo "[ERROR] ROS_ENV(${ROS_ENV}) variable is not set, cannot continue"
  exit 1
fi

# OVERLAYS to source come from MODULE
# shellcheck disable=SC2153
IFS=":" read -ra OVERLAYS_ARRAY <<< "${OVERLAYS}"

for OVERLAY in "${OVERLAYS_ARRAY[@]}"; do
  # shellcheck source=/dev/null
  source "${OVERLAY}"
done

#
if [[ -z "${CACHE_DIR}" || "${CACHE_DIR}" == "" ]]; then
  echo "[INFO] attempting to set CACHE_DIR to default VIAM_MODULE_DATA:${VIAM_MODULE_DATA}"
  export CACHE_DIR=${VIAM_MODULE_DATA}
  if [[ -z "${CACHE_DIR}" || "${CACHE_DIR}" == "" ]]; then
    echo "[ERROR] CACHE_DIR must be set, please see README"
    exit 1
  fi
fi

# ROS/VIAM variables
if [[ -z "${VIAM_NODE_NAME}" || "${VIAM_NODE_NAME}" == "" ]]; then
  echo "[WARNING] no VIAM_NODE_NAME specified, using viam_node by default"
  export VIAM_NODE_NAME="viam_node"
fi

if [[ -z "${VIAM_ROS_NAMESPACE}" || "${VIAM_ROS_NAMESPACE}" == "" ]]; then
  echo "[INFO] no ROS namespace provided, plase ensure this is correct"
fi

# viam tools needed for utils.py
RUST_UTILS_SO=$(find "${SCRIPT_DIR}" -name libviam_rust_utils.so -printf '%h')
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${RUST_UTILS_SO}

# execute script
exec "${SCRIPT_DIR}"/venv/bin/python3 "${SCRIPT_DIR}"/main.py "$@"
