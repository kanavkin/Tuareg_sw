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
# 500
printf '\x57\x00\x01\xF4' > $DEV
sleep 0.1

#dynamic dwell us
# 6000
printf '\x57\x01\x17\x70' > $DEV
sleep 0.1

#safety margin
# 15
printf '\x57\x02\x00\x0F' > $DEV
sleep 0.1

#idle ignition position
# POSITION_B2 -> 3
printf '\x57\x03\x00\x03' > $DEV
sleep 0.1

#idle dwell position
# POSITION_D2 -> 7
printf '\x57\x04\x00\x07' > $DEV
sleep 0.1

#idle advance deg
# 3
printf '\x57\x05\x00\x03' > $DEV
sleep 0.1

#idle dwell deg
# 190
printf '\x57\x06\x00\xBE' > $DEV
sleep 0.1

#print new values
printf "L" > $DEV



