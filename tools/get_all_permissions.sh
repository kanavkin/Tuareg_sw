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
printf "Jdec#" > $DEV
printf "Jtua#" > $DEV
printf "Jign#" > $DEV
printf "Jfue#" > $DEV

printf "Jbrn!" > $DEV
