#!/bin/sh

killall -SIGKILL rosmaster
killall -SIGKILL .shell_exec10A.sh
killall -SIGKILL python3
killall -SIGKILL rosout
pkill asv*