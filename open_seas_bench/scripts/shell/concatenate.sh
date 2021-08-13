#!/bin/sh


if [ $# -eq 0 ]; then
  echo "Usage : bash postprocessing.sh <file1> [file2] [...]"
else
#  awk 'NR>1' $@ | head -n2
  cd ../output
  awk '1' $@ |
  sort -nu -t' ' -k1,1 | 
  awk ' 
      { while((L+1) < $1) {print L+=1, "nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan" > "output.txt";  count+=1}} 
      { L=$1 } 
      1 {print > "output.txt"}
      END {print "Number of missing lines : " count}
      ' 
  cd ../input
  awk '1' $@ |
  sort -nu -t' ' -k1,1 | 
  awk ' 
      { while((L+1) < $1) {print L+=1, "nan nan nan nan nan nan nan nan nan nan nan nan nan nan nan" > "output.txt";  count+=1}} 
      { L=$1 } 
      1 {print > "output.txt"}
      '
fi 
