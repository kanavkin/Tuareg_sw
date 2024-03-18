#!/bin/bash

if [ "$1" != "" ];
then
    DEV=$1
else
    echo "no device specified, using /dev/tuareg"
    DEV="/dev/tuareg"
fi

#stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
#stty -F /dev/ttyUSB0 115200 raw

#get permissions
printf "Jfue#" > $DEV
printf "Jbrn!" > $DEV

#page 15 - P
printf '\x50\x31\x35' > $DEV

#values W%2o%v
printf '\x57\x00\x00\x00' > $DEV
printf '\x57\x00\x01\x00' > $DEV

printf '\x57\x00\x02\x00' > $DEV
printf '\x57\x00\x03\x00' > $DEV

printf '\x57\x00\x04\x00' > $DEV
printf '\x57\x00\x05\x00' > $DEV

printf '\x57\x00\x06\x00' > $DEV
printf '\x57\x00\x07\x00' > $DEV

