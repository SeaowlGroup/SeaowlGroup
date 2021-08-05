#!/bin/sh

cd ..
mkdir -p log
rm -f log/*
cd api
# nohup python3 executable9.py > log/nohup.out 2> log/nohup.err &
nohup ./.shell_exec10.sh 1 6 > ../log/nohup.out 2> ../log/nohup.err &
