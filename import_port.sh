#!/bin/bash

socat -d -d PTY,raw,echo=0,link=/dev/ttyUSB0 tcp:192.168.2.2:4000,nodelay,forever

exit 0