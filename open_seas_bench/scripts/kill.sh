#!/bin/sh

killall -SIGKILL rosmaster
killall -SIGKILL autoSim.sh
killall -SIGKILL python3
killall -SIGKILL rosout
pkill asv*