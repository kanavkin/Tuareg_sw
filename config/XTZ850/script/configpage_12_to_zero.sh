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

#set page  13 -> configpage 12
printf '\x50\x3d' > $DEV
sleep 0.1

#setting calib values:
#printf '\x57\x24\x00\x00' > $DEV
# "W" offset value_H value_L


#trigger map positions A1 .. D2
printf '\x57\x00\x00\x00' > $DEV
sleep 0.1
printf '\x57\x01\x00\x00' > $DEV
sleep 0.1
printf '\x57\x02\x00\x00' > $DEV
sleep 0.1
printf '\x57\x03\x00\x00' > $DEV
sleep 0.1
printf '\x57\x04\x00\x00' > $DEV
sleep 0.1
printf '\x57\x05\x00\x00' > $DEV
sleep 0.1
printf '\x57\x06\x00\x00' > $DEV
sleep 0.1
printf '\x57\x07\x00\x00' > $DEV
sleep 0.1

#decoder offset
printf '\x57\x08\x00\x00' > $DEV
sleep 0.1

#decoder delay
printf '\x57\x09\x00\x00' > $DEV
sleep 0.1

#crank noise filter
printf '\x57\x0A\x00\x00' > $DEV
sleep 0.1

#sync ratio min
printf '\x57\x0B\x00\x00' > $DEV
sleep 0.1

#sync ratio max
printf '\x57\x0C\x00\x00' > $DEV
sleep 0.1

#sync stability threshold
printf '\x57\x0D\x00\x00' > $DEV
sleep 0.1

#decoder timeout
printf '\x57\x0E\x00\x00' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



