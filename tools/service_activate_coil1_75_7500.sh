#!/bin/bash

if [ "$1" != "" ];
then
    DEV=$1
else
    echo "no device specified, using /dev/ttyACM0"
    DEV="/dev/ttyACM0"
fi

#stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
#stty -F /dev/ttyUSB0 115200 raw 

# M  0xFF 0xFF 0x00 0x00FF
printf '\x4D\x04\x06\x02\x1D\x4C' > $DEV



