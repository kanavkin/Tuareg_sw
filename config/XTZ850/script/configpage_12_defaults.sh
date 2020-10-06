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

printf "Jbrn!" > $DEV
#sleep 0.1

#set page  13 -> configpage 12
printf '\x50\x3d' > $DEV
sleep 0.1

#setting calib values:
#printf '\x57\x24\x00\x00' > $DEV
# "W" offset value_H value_L

#trigger map positions A1 .. D2

# 0
printf '\x57\x00\x00\x00' > $DEV
sleep 0.1

# 40
printf '\x57\x01\x00\x28' > $DEV
sleep 0.1

# 90
printf '\x57\x02\x00\x5A' > $DEV
sleep 0.1

# 98
printf '\x57\x03\x00\x62' > $DEV
sleep 0.1

# 180
printf '\x57\x04\x00\xB4' > $DEV
sleep 0.1

# 188
printf '\x57\x05\x00\xBC' > $DEV
sleep 0.1

# 270
printf '\x57\x06\x01\x0E' > $DEV
sleep 0.1

# 278
printf '\x57\x07\x01\x16' > $DEV
sleep 0.1

#decoder offset
# 260
printf '\x57\x08\x01\x04' > $DEV
sleep 0.1

#decoder delay
# 40
printf '\x57\x09\x00\x28' > $DEV
sleep 0.1

#crank noise filter
# 8
printf '\x57\x0A\x00\x08' > $DEV
sleep 0.1

#sync ratio min
# 30
printf '\x57\x0B\x00\x1E' > $DEV
sleep 0.1

#sync ratio max
# 50
printf '\x57\x0C\x00\x32' > $DEV
sleep 0.1

#sync stability threshold
#80
printf '\x57\x0D\x00\x50' > $DEV
sleep 0.1

#decoder timeout
# 3
printf '\x57\x0E\x00\x03' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



