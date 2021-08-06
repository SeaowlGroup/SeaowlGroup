#!/bin/sh

cd ..
mkdir -p log
rm -f log/*
cd scripts

if [ $# -le 1 ]; then
  echo "Usage : $0 <bench> <start_opus> <n_process> [serial]"
fi

# nohup python3 executable9.py > log/nohup.out 2> log/nohup.err &
nohup ./autoSim.sh $1 $2 $3 $4 > ../log/nohup.out 2> ../log/nohup.err &
