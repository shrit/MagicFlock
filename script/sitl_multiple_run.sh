#!/bin/bash

# Start several PX4 instances with
# Gazebo simulator. The objective is
# to make PX4 instances independent
# from ROS launch files. You should
# have already compiled PX4 in the
# Firmware This file is a modified
# version from the original
# sitl_multiple_run.sh in the
# Firmware/Tools directory.
# It can start up to 10 drones without
# the need for ROS in Gazebo simulator.

# The simulator is expected to send
# to UDP port 14560+i for i in [0,N-1]

# Please add the following line in your bashrc or zshrc file:
# FIRMWARE_DIR=/path/to/the/firmware

build_path=${FIRMWARE_DIR}/build/px4_sitl_default
src_path=${FIRMWARE_DIR}

sitl_num=2
[ -n "$1" ] && sitl_num="$1"

echo "killing running instances"

pkill -x px4 || true
pkill -x gazebo || true
pkill -x gzserver || true

# We need to wait more than 1 sec to
# free the TCP port. However, we
# need to find out how much
sleep 1

export PX4_SIM_MODEL=iris

n=0
while [ $n -lt $sitl_num ]; do
    working_dir="$build_path/instance_$n"
    [ ! -d "$working_dir" ] && mkdir -p "$working_dir"
    
    pushd "$working_dir" &>/dev/null
    echo "starting instance $n in $(pwd)"
    ../bin/px4 -i $n -d "$src_path/ROMFS/px4fmu_common" -s etc/init.d-posix/rcS >out.log 2>err.log &
    popd &>/dev/null
    
    n=$(($n + 1))
done

model="$2"

source "$src_path/Tools/setup_gazebo.bash" "${src_path}" "${build_path}"

gzserver --verbose "/meta/QCopters/envs/worlds/outdoor.world" &
SIM_PID=`echo $!`

if [[ -n "$HEADLESS" ]]; then
    echo "not running gazebo gui"
else
    # gzserver needs to be running to avoid a race. Since the launch
    # is putting it into the background we need to avoid it by backing off
    sleep 3
    nice -n 20 gzclient --verbose &
    GUI_PID=`echo $!`
fi
