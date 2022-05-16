#!/bin/bash

export LD_LIBRARY_PATH=/opt/stlink

ST=/opt/stlink/st-flash

if [ "$1" != "" ];
then
    TUAREG=$1
else
    echo "no version specified"
    exit 1
fi

#st-flash write ../Releases/$TUAREG/Tuareg.bin 0x8000000
$ST  write ../Releases/$TUAREG/Tuareg.bin 0x8000000





