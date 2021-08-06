#!/bin/sh

n=$(wc -l "$(ps -A | grep python3)")

if [ ${n} > 1 ]; then
  echo "yup"
else
  echo "nop"
fi
