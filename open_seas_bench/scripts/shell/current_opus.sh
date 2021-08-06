#!/bin/sh

cd ../output
cur=$(tail -1 "$(ls sur* | tail -1)" | cut -d ' ' -f 1)
echo ${cur}
