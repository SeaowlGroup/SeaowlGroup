#!/bin/sh

process=$(pstree | grep python3)
n=0
if [ -s process ]; then
  n=$(wc -l ${process})
fi

if [ ${n} -ge 1 ]; then
  echo "yup"
else
  echo "nop"
fi
