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

#setting calib values:
#printf '\x55\x00\x01\x00\x00\x00\x00' > $DEV
# "U" "offset_H" offset_L value_MSB value_3 value_2 value_LSB

#print old values
printf "L" > $DEV

#trigger map positions A1 .. D2
# 0
printf '\x55\x00\x00\x00\x00\x00\x00' > $DEV
sleep 0.2

# 40
printf '\x55\x00\x01\x00\x00\x00\x28' > $DEV
sleep 0.2

# 90
printf '\x55\x00\x02\x00\x00\x00\x5A' > $DEV
sleep 0.2

# 98
printf '\x55\x00\x03\x00\x00\x00\x62' > $DEV
sleep 0.2

# 180
printf '\x55\x00\x04\x00\x00\x00\xB4' > $DEV
sleep 0.2

# 188
printf '\x55\x00\x05\x00\x00\x00\xBC' > $DEV
sleep 0.2

# 270
printf '\x55\x00\x06\x00\x00\x01\x0E' > $DEV
sleep 0.2

# 278
printf '\x55\x00\x07\x00\x00\x01\x16' > $DEV
sleep 0.2


#decoder offset
# 260
printf '\x55\x00\x08\x00\x00\x01\x04' > $DEV
sleep 0.2

#decoder delay
# 40
printf '\x55\x00\x09\x00\x00\x00\x28' > $DEV
sleep 0.2

#crank noise filter
# 8
printf '\x55\x00\x0A\x00\x00\x00\x08' > $DEV
sleep 0.2

#sync ratio min
# 30
printf '\x55\x00\x0B\x00\x00\x00\x1E' > $DEV
sleep 0.2

#sync ratio max
# 50
printf '\x55\x00\x0C\x00\x00\x00\x32' > $DEV
sleep 0.2

#decoder timeout
#3
printf '\x55\x00\x0D\x00\x00\x00\x03' > $DEV
sleep 0.2


#print new values
printf "L" > $DEV
