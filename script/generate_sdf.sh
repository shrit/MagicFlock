#!/bin/bash
## script to generate sdf files and apply  wireless sensors on them

if [ "$1" == "-h" ] || [ "$1" == "--help" ]
then
	echo "Usage: $0 [-n <num_vehicles>] [-m <vehicle_model>]"
	exit 1
fi

while getopts n:m: option
do
	case "${option}"
	in
		n) NUM_VEHICLES=${OPTARG};;
		m) VEHICLE_MODEL=${OPTARG};;
	esac
done

num_vehicles=${NUM_VEHICLES:=3}
export PX4_SIM_MODEL=${VEHICLE_MODEL:=iris}

n=0
while [ $n -lt $num_vehicles ]; do

	python3 xacro.py rotors_description/urdf/${PX4_SIM_MODEL}_base.xacro \
		rotors_description_dir:=rotors_description mavlink_udp_port:=$(($mavlink_udp_port+$n))\
		mavlink_tcp_port:=$(($mavlink_tcp_port+$n))  -o sdf/${PX4_SIM_MODEL}_${n}.urdf

	gz sdf -p  sdf/${PX4_SIM_MODEL}_${n}.urdf > sdf/${PX4_SIM_MODEL}_${n}.sdf
	#sed -i '2 r file3.txt' file1.txt
	n=$(($n + 1))
done

