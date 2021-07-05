#!/bin/sh

echo "Enter either the number of the output to delete, or 'all', or 'last' : "
read file
if [ "$file" = "all" ]; then
  rm ../output/*
elif [ "$file" = "last" ]; then
  cd ../output
  rm "$(ls | tail -1)"
  cd ../scripts
else
  rm ../output/"$file"*
fi
