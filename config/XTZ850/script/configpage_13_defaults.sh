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

printf "Jign#" > $DEV
sleep 0.1

printf "Jbrn!" > $DEV
sleep 0.1

#set page  14 -> configpage 13
printf '\x50\x3e' > $DEV
sleep 0.1


#setting config values:
#printf '\x55\x00\x01\x00\x00\x00\x00' > $DEV
# "U" "offset_H" offset_L value_MSB value_3 value_2 value_LSB


# max_rpm
# 9000
printf '\x55\x00\x00\x00\x00\x23\x28' > $DEV
sleep 0.1

# dynamic_min_rpm
# 500
printf '\x55\x00\x01\x00\x00\x01\xF4' > $DEV
sleep 0.1

# dynamic_ignition_base_position
# CRK_POSITION_A2 -> 1
printf '\x55\x00\x02\x00\x00\x00\x01' > $DEV
sleep 0.1

# dynamic_dwell_base_position
# CRK_POSITION_B2 -> 3
printf '\x55\x00\x03\x00\x00\x00\x03' > $DEV
sleep 0.1

# dynamic_dwell_target_us
# 6000
printf '\x55\x00\x04\x00\x00\x17\x70' > $DEV
sleep 0.1

# cold_idle_cutoff_rpm
# 1500
printf '\x55\x00\x05\x00\x00\x05\xDC' > $DEV
sleep 0.1

# cold_idle_cutoff_CLT_K
# (50 deg C) 323
printf '\x55\x00\x06\x00\x00\x01\x43' > $DEV
sleep 0.1

# cold_idle_ignition_advance_deg
# 20
printf '\x55\x00\x07\x00\x00\x00\x14' > $DEV
sleep 0.1

# cold_idle_dwell_target_us
# 10000
printf '\x55\x00\x08\x00\x00\x27\x10' > $DEV
sleep 0.1

# cranking_ignition_position
# POSITION_B2 -> 3
printf '\x55\x00\x09\x00\x00\x00\x03' > $DEV
sleep 0.1

# cranking_dwell_position
# POSITION_D2 -> 7
printf '\x55\x00\x0A\x00\x00\x00\x07' > $DEV
sleep 0.1


#print new values
printf "L" > $DEV



