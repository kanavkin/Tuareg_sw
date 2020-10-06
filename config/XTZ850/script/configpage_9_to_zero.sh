#!/bin/bash

DEV="/dev/ttyUSB0"

#stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
#stty -F /dev/ttyUSB0 115200 raw 

#get permissions
printf "Jcal#" > $DEV
printf "Jbrn!" > $DEV

#set config page 9 --> 12
printf '\x50\x3c' > $DEV

#setting calib values:
#printf '\x45\x42\x00\x00\x00\x00' > $DEV
# "W" offset value_H value_M1 value_m2 value_L


#IAT M
printf '\x45\x00\x00\x00\x00\x00' > $DEV
sleep 0.1

#IAT N
printf '\x45\x01\x00\x00\x00\x00' > $DEV
sleep 0.1

#CLT M
printf '\x45\x02\x00\x00\x00\x00' > $DEV
sleep 0.1

#CLT N
printf '\x45\x03\x00\x00\x00\x00' > $DEV
sleep 0.1


#TPS_calib_M
printf '\x45\x04\x00\x00\x00\x00' > $DEV
sleep 0.1

#TPS_calib_N
printf '\x45\x05\x00\x00\x00\x00' > $DEV
sleep 0.1


#MAP_calib_M
printf '\x45\x06\x00\x00\x00\x00' > $DEV
sleep 0.1

#MAP_calib_N
printf '\x45\x07\x00\x00\x00\x00' > $DEV
sleep 0.1


#BARO_calib_M
printf '\x45\x08\x00\x00\x00\x00' > $DEV
sleep 0.1

#BARO_calib_N
printf '\x45\x09\x00\x00\x00\x00' > $DEV
sleep 0.1


#O2_calib_M
printf '\x45\x0A\x00\x00\x00\x00' > $DEV
sleep 0.1

#O2_calib_N
printf '\x45\x0B\x00\x00\x00\x00' > $DEV
sleep 0.1


#VBAT_calib_M
printf '\x45\x0C\x00\x00\x00\x00' > $DEV
sleep 0.1

#VBAT_calib_N
printf '\x45\x0D\x00\x00\x00\x00' > $DEV
sleep 0.1


#KNOCK_calib_M
printf '\x45\x0E\x00\x00\x00\x00' > $DEV
sleep 0.1

#KNOCK_calib_N
printf '\x45\x0F\x00\x00\x00\x00' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



