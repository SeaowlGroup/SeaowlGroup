#!/bin/sh

cd ..
mkdir -p log
rm -f log/*
cd api
# nohup python3 executable9.py > log/nohup.out 2> log/nohup.err &
nohup ./.shell_exec10A.sh 13096 6 dataAdrien.txt > ../log/nohup.out 2> ../log/nohup.err &
