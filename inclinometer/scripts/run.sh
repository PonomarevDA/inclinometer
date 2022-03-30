#!/bin/bash

EXPECTED_VID=0483
EXPECTED_PID=374b
EXPECTED_DEV_PATH="/dev/ttyACM*"

for dev_path in $EXPECTED_DEV_PATH; do
    check_vid_and_pid=$(udevadm info $dev_path |
                        grep -E "(ID_MODEL_ID=$EXPECTED_PID|ID_VENDOR_ID=$EXPECTED_VID)" -wc)
    if [ "$check_vid_and_pid" == 2 ]
    then
        DEV_PATH=$dev_path
    fi
done

if [ -z $DEV_PATH ]
then
    echo "Can't find expected tty device."
    exit
else
    echo $DEV_PATH
    ./fake_gps.py $DEV_PATH
fi
