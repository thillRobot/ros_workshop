#!/bin/bash
set -e
source /opt/ros/noetic/setup.bash

touch /entrypoint_test


exec "$@"