#!/bin/bash

ST=/opt/stlink/st-upload.sh

if [ "$1" != "" ];
then
    TUAREG=$1
else
    echo "no version specified"
    exit 1
fi

$ST ../Releases/$TUAREG/Tuareg.bin




