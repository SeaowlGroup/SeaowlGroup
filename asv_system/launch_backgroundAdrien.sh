#!/bin/sh

# nohup python3 executable9.py > /dev/null &
# nohup python3 executable9.py > log/nohup.out 2> log/nohup.err &
nohup python3 executable9Adrien.py 2> log/nohup.err & 
