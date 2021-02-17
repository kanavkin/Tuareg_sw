#!/bin/bash

# run with /bin/bash !!!

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
printf "Jmod#" > $DEV
sleep 0.1
printf "Jdec#" > $DEV
sleep 0.1

printf "Jbrn!" > $DEV
sleep 0.1

#set page  13 -> configpage 12
printf '\x50\x3d' > $DEV
sleep 0.1

#setting config values:
#printf '\x55\x00\x01\x00\x00\x00\x00' > $DEV
# "U" "offset_H" offset_L value_MSB value_3 value_2 value_LSB


#trigger map positions A1 .. D2
printf '\x55\x00\x00\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x01\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x02\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x03\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x04\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x05\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x06\x00\x00\x00\x00' > $DEV
sleep 0.1
printf '\x55\x00\x07\x00\x00\x00\x00' > $DEV
sleep 0.1

#decoder offset
printf '\x55\x00\x08\x00\x00\x00\x00' > $DEV
sleep 0.1

#decoder delay
printf '\x55\x00\x09\x00\x00\x00\x00' > $DEV
sleep 0.1

#crank noise filter
printf '\x55\x00\x0A\x00\x00\x00\x00' > $DEV
sleep 0.1

#sync ratio min
printf '\x55\x00\x0B\x00\x00\x00\x00' > $DEV
sleep 0.1

#sync ratio max
printf '\x55\x00\x0C\x00\x00\x00\x00' > $DEV
sleep 0.1

#decoder timeout
printf '\x55\x00\x0D\x00\x00\x00\x00' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



