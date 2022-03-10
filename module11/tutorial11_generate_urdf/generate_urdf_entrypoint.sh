#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash
source /generate_urdf_workspace/devel/setup.bash

touch /entrypoint_test

exec "$@"