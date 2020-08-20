#!/bin/bash

DEV="/dev/ttyUSB0"

#stty -F /dev/ttyUSB0 115200 raw -echo -hupcl
#stty -F /dev/ttyUSB0 115200 raw 


#get modify and write permissions
printf "Jmod#" > $DEV
sleep 0.1
printf "Jbrn!" > $DEV
sleep 0.1

#set config page 9 --> 12
printf '\x50\x3c' > $DEV


#setting calib values:
#printf '\x57\x24\x00\x00' > $DEV
# "W" offset value_H value_L


#IAT x 0..5
printf '\x57\x00\x00\x00' > $DEV
sleep 0.1
printf '\x57\x01\x03\x33' > $DEV
sleep 0.1
printf '\x57\x02\x06\x66' > $DEV
sleep 0.1
printf '\x57\x03\x09\x99' > $DEV
sleep 0.1
printf '\x57\x04\x0C\xCC' > $DEV
sleep 0.1
printf '\x57\x05\x0F\xFF' > $DEV
sleep 0.1

#IAT y 0..5
printf '\x57\x06\x00\x00' > $DEV
sleep 0.1
printf '\x57\x07\x00\x00' > $DEV
sleep 0.1
printf '\x57\x08\x00\x00' > $DEV
sleep 0.1
printf '\x57\x09\x00\x00' > $DEV
sleep 0.1
printf '\x57\x0A\x00\x00' > $DEV
sleep 0.1
printf '\x57\x0B\x00\x00' > $DEV
sleep 0.1



#CLT x 0..5
printf '\x57\x0C\x00\x00' > $DEV
sleep 0.1
printf '\x57\x0D\x03\x33' > $DEV
sleep 0.1
printf '\x57\x0E\x06\x66' > $DEV
sleep 0.1
printf '\x57\x0F\x09\x99' > $DEV
sleep 0.1
printf '\x57\x10\x0C\xCC' > $DEV
sleep 0.1
printf '\x57\x11\x0F\xFF' > $DEV
sleep 0.1

#CLT y 0..5
printf '\x57\x12\x00\x00' > $DEV
sleep 0.1
printf '\x57\x13\x00\x00' > $DEV
sleep 0.1
printf '\x57\x14\x00\x00' > $DEV
sleep 0.1
printf '\x57\x15\x00\x00' > $DEV
sleep 0.1
printf '\x57\x16\x00\x00' > $DEV
sleep 0.1
printf '\x57\x17\x00\x00' > $DEV
sleep 0.1


#TPS_calib_M
#25
printf '\x57\x18\x00\x19' > $DEV
sleep 0.1

#TPS_calib_N
#159
printf '\x57\x19\x00\x9F' > $DEV
sleep 0.1

#MAP_calib_M
printf '\x57\x1A\x00\x01' > $DEV
sleep 0.1

#MAP_calib_N
printf '\x57\x1B\x00\x00' > $DEV
sleep 0.1

#MAP_calib_L
printf '\x57\x1E\x00\x01' > $DEV
sleep 0.1



#BARO_calib_M
printf '\x57\x1F\x00\x01' > $DEV
sleep 0.1

#BARO_calib_N
printf '\x57\x20\x00\x00' > $DEV
sleep 0.1

#BARO_calib_L
printf '\x57\x21\x00\x01' > $DEV
sleep 0.1



#O2_calib_M
printf '\x57\x22\x00\x01' > $DEV
sleep 0.1

#O2_calib_N
printf '\x57\x23\x00\x00' > $DEV
sleep 0.1

#O2_calib_L
printf '\x57\x24\x00\x01' > $DEV
sleep 0.1



#VBAT_calib_M
printf '\x57\x25\x00\x01' > $DEV
sleep 0.1

#VBAT_calib_N
printf '\x57\x26\x00\x00' > $DEV
sleep 0.1

#VBAT_calib_L
printf '\x57\x27\x00\x01' > $DEV
sleep 0.1



#KNOCK_calib_M
printf '\x57\x28\x00\x01' > $DEV
sleep 0.1

#KNOCK_calib_N
printf '\x57\x29\x00\x00' > $DEV
sleep 0.1

#KNOCK_calib_L
printf '\x57\x2A\x00\x01' > $DEV
sleep 0.1



#print new values
printf "L" > $DEV



