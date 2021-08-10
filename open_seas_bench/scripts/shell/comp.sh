#!/bin/sh

if [ $# -le 1 ]; then
  echo "Usage : $0 <file1> <file2>"

else
    f1=$1
    f2=$2
    awk '{print $1}' $f1 | sort -n  > comp1
    awk '{print $1}' $f2 | sort -n  > comp2
    echo "\n comp1 \n"
    uniq -d comp1
    echo "\n comp2 \n"
    uniq -d comp2
    sort -nC comp1
    sort -nC comp2
    comm -3 comp1 comp2
fi