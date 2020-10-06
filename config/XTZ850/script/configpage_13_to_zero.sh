#!/bin/bash

# run with /bin/bash !!!

DEV="/dev/ttyUSB0"

#stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
#stty -F /dev/ttyUSB0 115200 raw 

#get permissions
printf "Jmod#" > $DEV
sleep 0.1

#printf "Jbrn!" > $DEV
#sleep 0.1

#set page  14 -> configpage 13
printf '\x50\x3e' > $DEV
sleep 0.1

#setting calib values:
#printf '\x57\x24\x00\x00' > $DEV
# "W" offset value_H value_L


#dynamic min rpm
printf '\x57\x00\x00\x00' > $DEV
sleep 0.1

#dynamic dwell us
printf '\x57\x01\x00\x00' > $DEV
sleep 0.1

#safety margin
printf '\x57\x02\x00\x00' > $DEV
sleep 0.1

#idle ignition position
printf '\x57\x03\x00\x00' > $DEV
sleep 0.1

#idle dwell position
printf '\x57\x04\x00\x00' > $DEV
sleep 0.1

#idle advance deg
printf '\x57\x05\x00\x00' > $DEV
sleep 0.1

#idle dwell deg
printf '\x57\x06\x00\x00' > $DEV
sleep 0.1

#print new values
printf "L" > $DEV



