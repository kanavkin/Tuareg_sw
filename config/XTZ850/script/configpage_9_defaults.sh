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
#printf '\x45\x42\x00\x00\x00\x00' > $DEV
# "W" offset value_H value_M1 value_m2 value_L


#IAT_calib_M
#27,5 0x41dc0000
printf '\x45\x00\x41\xdc\x00\x00' > $DEV
sleep 0.1

#IAT_calib_N
# 587 0x4412c000
printf '\x45\x01\x44\x12\xc0\x00' > $DEV
sleep 0.1

#CLT_calib_M
#27,5 0x41dc0000
printf '\x45\x02\x41\xdc\x00\x00' > $DEV
sleep 0.1

#CLT_calib_N
#600 0x44160000
printf '\x45\x03\x44\x16\x00\x00' > $DEV
sleep 0.1

#TPS_calib_M
#25 0x41c80000
printf '\x45\x04\x41\xc8\x00\x00' > $DEV
sleep 0.1

#TPS_calib_N
#159 0x431f0000
printf '\x45\x05\x43\x1f\x00\x00' > $DEV
sleep 0.1

#MAP_calib_M
#18,8 0x41966666
printf '\x45\x06\x41\x96\x66\x66' > $DEV
sleep 0.1

#MAP_calib_N
#359 0x43b38000
printf '\x45\x07\x43\xb3\x00\x00' > $DEV
sleep 0.1

#BARO_calib_M
#18,65 0x41953333
printf '\x45\x08\x41\x95\x33\x33' > $DEV
sleep 0.1

#BARO_calib_N
#359 0x43b38000
printf '\x45\x09\x43\xb3\x80\x00' > $DEV
sleep 0.1

#O2_calib_M
# 1 0x3f800000
printf '\x45\x0A\x3f\x80\x00\x00' > $DEV
sleep 0.1

#O2_calib_N
#0
printf '\x45\x0B\x00\x00\x00\x00' > $DEV
sleep 0.1


#VBAT_calib_M
#209 0x43510000
printf '\x45\x0C\x43\x51\x00\x00' > $DEV
sleep 0.1

#VBAT_calib_N
#-141 0xc30d0000
printf '\x45\x0D\xc3\x0d\x00\x00' > $DEV
sleep 0.1


#KNOCK_calib_M
#1 0x3f800000
printf '\x45\x0E\x3f\x80\x00\x00' > $DEV
sleep 0.1

#KNOCK_calib_N
#0
printf '\x45\x0F\x00\x00\x00\x00' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



