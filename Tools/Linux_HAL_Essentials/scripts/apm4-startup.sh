#!/bin/bash

# Script location

LOCATION="/root/ardupilot/Tools/Linux_HAL_Essentials"

echo "Loading Test_Capes..."
cp $LOCATION/BB-SPI0-PXF-01-00A0.dtbo /lib/firmware/
cp $LOCATION/BB-SPI1-PXF-01-00A0.dtbo /lib/firmware/
cp $LOCATION/BB-BONE-PRU-05-00A0.dtbo /lib/firmware/
cp $LOCATION/rcinpru0 /lib/firmware
cp $LOCATION/pwmpru1 /lib/firmware

# Loading the capes
echo BB-BONE-PRU-05 > /sys/devices/bone_capemgr.*/slots
echo BB-SPI0-PXF-01 > /sys/devices/bone_capemgr.*/slots
echo BB-SPI1-PXF-01 > /sys/devices/bone_capemgr.*/slots
echo BB-UART5 > /sys/devices/bone_capemgr.*/slots
echo BB-UART4 > /sys/devices/bone_capemgr.*/slots
echo BB-UART2 > /sys/devices/bone_capemgr.*/slots

# Set CPU at max speed
cpufreq-set -f 1000MHz

# Line for making PREEMPT_RT work
echo 0:rcinpru0 > /sys/devices/ocp.3/4a300000.prurproc/load

# Logging
dmesg | grep "SPI"
dmesg | grep "PRU"
cat /sys/devices/bone_capemgr.*/slots

cd /root
(
    date
    init 3
    #killall -q udhcpd
    while :; do
	# Start copter, modify if other vehicle is needed
        #./ArduCopter.elf -A /dev/ttyO0 -B /dev/ttyO5
	./ArduCopter.elf -A tcp:*:6000:wait -B /dev/ttyO5
    done
) >> ~/copter.log 2>&1

