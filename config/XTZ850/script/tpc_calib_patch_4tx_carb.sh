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
# value in IEEE-754 float format
# gdb --batch -ex "print/x (float *) ( (float) 2150 )"

#TPS_calib_M
#was 25 0x41c80000
# -25 0xc1c80000
#printf '\x45\x04\x41\xc8\x00\x00' > $DEV
printf '\x45\x04\xc1\xc8\x00\x00' > $DEV
sleep 0.1

#TPS_calib_N
# N := TPS_raw at full throttle + 2250
# 589 + 2250 = 2839

#2150 0x45066000
printf '\x45\x05\x45\x06\x60\x00' > $DEV
sleep 0.1

