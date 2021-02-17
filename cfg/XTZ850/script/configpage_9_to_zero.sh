#!/bin/bash

if [ "$1" != "" ];
then
    DEV=$1
else
    echo "no device specified, using /dev/ttyUSB0"
    DEV="/dev/ttyUSB0"
fi

#stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
#stty -F /dev/ttyUSB0 115200 raw

#get permissions
printf "Jcal#" > $DEV
printf "Jbrn!" > $DEV

#set config page 9 --> 12
printf '\x50\x3c' > $DEV

#setting calib values:
#printf '\x55\x00\x01\x00\x00\x00\x00' > $DEV
# "U" "offset_H" offset_L value_MSB value_3 value_2 value_LSB

#IAT M
printf '\x55\x00\x00\x00\x00\x00\x00' > $DEV
sleep 0.1

#IAT N
printf '\x55\x00\x01\x00\x00\x00\x00' > $DEV
sleep 0.1

#CLT M
printf '\x55\x00\x02\x00\x00\x00\x00' > $DEV
sleep 0.1

#CLT N
printf '\x55\x00\x03\x00\x00\x00\x00' > $DEV
sleep 0.1


#TPS_calib_M
printf '\x55\x00\x04\x00\x00\x00\x00' > $DEV
sleep 0.1

#TPS_calib_N
printf '\x55\x00\x05\x00\x00\x00\x00' > $DEV
sleep 0.1


#MAP_calib_M
printf '\x55\x00\x06\x00\x00\x00\x00' > $DEV
sleep 0.1

#MAP_calib_N
printf '\x55\x00\x07\x00\x00\x00\x00' > $DEV
sleep 0.1


#BARO_calib_M
printf '\x55\x00\x08\x00\x00\x00\x00' > $DEV
sleep 0.1

#BARO_calib_N
printf '\x55\x00\x09\x00\x00\x00\x00' > $DEV
sleep 0.1


#O2_calib_M
printf '\x55\x00\x0A\x00\x00\x00\x00' > $DEV
sleep 0.1

#O2_calib_N
printf '\x55\x00\x0B\x00\x00\x00\x00' > $DEV
sleep 0.1


#VBAT_calib_M
printf '\x55\x00\x0C\x00\x00\x00\x00' > $DEV
sleep 0.1

#VBAT_calib_N
printf '\x55\x00\x0D\x00\x00\x00\x00' > $DEV
sleep 0.1


#KNOCK_calib_M
printf '\x55\x00\x0E\x00\x00\x00\x00' > $DEV
sleep 0.1

#KNOCK_calib_N
printf '\x55\x00\x0F\x00\x00\x00\x00' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



