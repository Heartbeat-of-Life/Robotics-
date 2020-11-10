#!/bin/bash
set -e #To exit on error (see also zsh --help)
source /opt/ros/foxy/setup.sh
export _colcon_cd_root=/opt/ros/foxy
#source /usr/share/colcon_cd/function/colcon_cd.sh

exec "$@"
