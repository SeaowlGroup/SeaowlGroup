#!/bin/sh

echo "Enter either the number of the output to delete, or 'all', or 'last' : "
read file
if [ "$file" = "all" ]; then
  rm ../output/21*
elif [ "$file" = "last" ]; then
  cd ../output
  rm "$(ls 21* | tail -1)"
  cd ../scripts
else
  rm ../output/"$file"*
fi
