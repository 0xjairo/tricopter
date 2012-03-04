#!/bin/sh
SCREENLOG_FILE=screenlog.0

if [ -e $SCREENLOG_FILE ]; then
    rm screenlog.0
fi

screen -L /dev/ttyACM0

