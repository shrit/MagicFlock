#!/bin/bash
# run multiple instances of the 'px4' binary, with the gazebo SITL simulation
# It assumes px4 is already built, with 'make px4_sitl_default'

# The simulator is expected to send to TCP port 4560+i for i in [0, N-1]
# For example gazebo can be run like this:
#./Tools/gazebo_sitl_multiple_run.sh -n 10 -m iris
#
#
# This file is forked from Firmware/Tools/gazebo_sitl_multiple_run.sh
# I have added only necessary information to be compatible with WiFi antenna
# Finally it starts gazebo client.

function cleanup() {
	pkill -x px4
	pkill gzserver
	pkill gzclient
}

function spawn_model() {
	MODEL=$1
	N=$2 #Instance Number

	SUPPORTED_MODELS=("iris" "iris_rtps" "plane" "standard_vtol" "rover" "r1_rover")
	if [[ " ${SUPPORTED_MODELS[*]} " != *"$MODEL "* ]];
	then
		echo "ERROR: Currently only vehicle model $MODEL is not supported!"
		echo "       Supported Models: [${SUPPORTED_MODELS[@]}]"
		trap "cleanup" SIGINT SIGTERM EXIT
		exit 1
	fi

	working_dir="$build_path/instance_$n"
	[ ! -d "$working_dir" ] && mkdir -p "$working_dir"

	pushd "$working_dir" &>/dev/null
	echo "starting instance $N in $(pwd)"
	../bin/px4 -i $N -d "$build_path/etc" -w sitl_${MODEL}_${N} -s etc/init.d-posix/rcS >out.log 2>err.log &
	python3 ${src_path}/Tools/sitl_gazebo/scripts/jinja_gen.py ${src_path}/Tools/sitl_gazebo/models/${MODEL}/${MODEL}.sdf.jinja ${src_path}/Tools/sitl_gazebo --mavlink_tcp_port $((4560+${N})) --mavlink_udp_port $((14560+${N})) --output-file /tmp/${MODEL}_${N}.sdf

	sed -i "347 r ${project_path}/sdf/wireless.sdf" ${project_path}/sdf/${MODEL}_${n}.sdf
	sed -i -e "s/osrf/${MODEL}_${n}/g"  ${project_path}/sdf/${MODEL}_${n}.sdf

	echo "Spawning ${MODEL}_${N}"

	gz model --spawn-file=/tmp/${MODEL}_${N}.sdf --model-name=${MODEL}_${N} -x 0.0 -y $((3*${N})) -z 0.0

	popd &>/dev/null

}

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>] [-w <world>] [-s <script>]"
	echo "-s flag is used to script spawning vehicles e.g. $0 -s iris:3,plane:2"
	exit 1
fi

while getopts n:m:w:s:t: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
		w) WORLD=${OPTARG};;
		s) SCRIPT=${OPTARG};;
		t) TARGET=${OPTARG};;
	esac
done
num_vehicles=${NUM_VEHICLES:=3}
world=${WORLD:=empty}
target=${TARGET:=px4_sitl_default}
export PX4_SIM_MODEL=${VEHICLE_MODEL:=iris}

echo ${SCRIPT}
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
src_path="/meta/lemon/lib/Firmware"

build_path=${src_path}/build/px4_sitl_default
mavlink_udp_port=14560
mavlink_tcp_port=4560
world="outdoor"

project_path=/meta/lemon/script

echo "killing running instances"
pkill -x px4 || true

sleep 1

source ${src_path}/Tools/setup_gazebo.bash ${src_path} ${src_path}/build/${target}

echo "Starting gazebo"
gzserver ${project_path}/worlds/${world}.world --verbose &
sleep 5

n=0
if [ -z ${SCRIPT} ]; then
	if [ $num_vehicles -gt 255 ]
	then
		echo "Tried spawning $num_vehicles vehicles. The maximum number of supported vehicles is 255"
		exit 1
	fi

	while [ $n -lt $num_vehicles ]; do
		spawn_model ${PX4_SIM_MODEL} $n
		n=$(($n + 1))
	done
else
	IFS=,
	for target in ${SCRIPT}; do
		target="$(echo "$target" | tr -d ' ')" #Remove spaces
		target_vehicle="${target%:*}"
		target_number="${target#*:}"

		if [ $n -gt 255 ]
		then
			echo "Tried spawning $n vehicles. The maximum number of supported vehicles is 255"
			exit 1
		fi

		m=0
		while [ $m -lt ${target_number} ]; do
			spawn_model ${target_vehicle} $n
			m=$(($m + 1))
			n=$(($n + 1))
		done
	done

fi
trap "cleanup" SIGINT SIGTERM EXIT

echo "Starting gazebo client"
gzclient > /dev/null 2>&1
