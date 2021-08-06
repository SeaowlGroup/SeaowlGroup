#!/bin/sh

echo "Enter either the number of the simulation to delete, or 'all', or 'last' : "
read file

while [ -z "$file" ]; do
  echo "Enter either the number of the simulation to delete, or 'all', or 'last' : "
  read file
done

cd ..

if [ "$file" = "all" ]; then
  rm -f output/21*
  rm -f input/21*
  rm -f log/nohup.out
  rm -f log/nohup.err
elif [ "$file" = "last" ]; then
  cd output
  rm -f "$(ls 21* | tail -1)"
  cd ../input
  rm -f "$(ls 21* | tail -1)"
  cd ../scripts
  rm -f ../log/nohup.out
  rm -f ../log/nohup.err
else
  rm -f output/"$file"*
  rm -f input/"$file"*
fi

rm -rf __pycache__
