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

# value in IEEE-754 float format
# gdb --batch -ex "print/x (float *) ( (float) 2150 )"

#TPS_calib_M
# -29 0xc1e80000
printf '\x55\x00\x04\xc1\xe8\x00\x00' > $DEV
sleep 0.1

#TPS_calib_N
# 3300 0x454e4000
printf '\x55\x00\x05\x45\x4e\x40\x00' > $DEV
sleep 0.1

