#!/bin/sh

cd ..
# nohup python3 executable9.py > log/nohup.out 2> log/nohup.err &
nohup ./.shell_exec10.sh 1 219999999999 > log/nohup.out 2> log/nohup.err &
