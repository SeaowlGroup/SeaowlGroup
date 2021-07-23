#!/bin/sh

echo "Enter either the number of the simulation to delete, or 'all', or 'last' : "
read file

while [ -z "$file" ]; do
  echo "Enter either the number of the simulation to delete, or 'all', or 'last' : "
  read file
done

if [ "$file" = "all" ]; then
  rm -f ../output/21*
  rm -f ../input/21*
elif [ "$file" = "last" ]; then
  cd ../output
  rm -f "$(ls 21* | tail -1)"
  cd ../input
  rm -f "$(ls 21* | tail -1)"
  cd ../scripts
else
  rm -f ../output/"$file"*
  rm -f ../input/"$file"*
fi
