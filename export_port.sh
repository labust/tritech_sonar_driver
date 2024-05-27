#!/bin/bash

socat -d -d /dev/ttyUSB0,raw,echo=0,b115200 tcp4-listen:4000,fork,reuseaddr

exit 0