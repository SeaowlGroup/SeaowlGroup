#!/bin/sh

echo "Enter either the number of the simulation to delete, or 'all', or 'last' : "
read file

while [ -z "$file" ]; do
  echo "Enter either the number of the simulation to delete, or 'all', or 'last' : "
  read file
done

if [ "$file" = "all" ]; then
  rm ../output/21*
  rm ../input/21*
elif [ "$file" = "last" ]; then
  cd ../output
  rm "$(ls 21* | tail -1)"
  cd ../input
  rm "$(ls 21* | tail -1)"
  cd ../scripts
else
  rm ../output/"$file"*
  rm ../input/"$file"*
fi
