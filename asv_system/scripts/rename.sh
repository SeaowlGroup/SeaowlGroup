#!/bin/sh

if [ "$#" = 0 ]; then
  while [ -z "$file" ]; do
    echo "Which name for the last simulation computed ?"
    read file
  done
  cd ../output
  mv "$(ls 21* | tail -1)" "${file}.txt"
  cd ../input
  mv "$(ls 21* | tail -1)" "${file}.txt"
elif [ "$#" = 1 ]; then
  cd ../output
  mv "$(ls 21* | tail -1)" "${1}.txt"
  cd ../input
  mv "$(ls 21* | tail -1)" "${1}.txt"
elif [ "$#" = 2 ]; then
  cd ../output
  mv "${1}.txt" "${2}.txt"
  cd ../input
  mv "${1}.txt" "${2}.txt"
fi
