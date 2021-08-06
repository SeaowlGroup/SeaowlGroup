#!/bin/sh

# nohup python3 executable9.py > /dev/null &
# nohup python3 executable9.py > log/nohup.out 2> log/nohup.err &

if [ $# -le 1 ]; then
  echo "Usage : $0 <bench> <start_opus> <n_process> [serial]"
else
  start_opus=$2
  n_process=$3
  serial=$4

  end_opus=$((${start_opus}+${n_process}-1))

  while [ ${start_opus} -le 50000 ]
  do
    # res=$(python3 executable10_aux.py ${start_opus} ${end_opus} ${serial})
    # res=`python3 -c 'import executable10_aux2; import io; import sys; sys.stdout = io.BytesIO(); end = executable10_aux2.go('${start_opus}', '${end_opus}', '${serial}'); end'`
    # res=$(python3 executable10_aux2.py ${start_opus} ${end_opus} ${serial} | rev | cut -f 1 -d ' ' | rev)

    python3 ../api/autoSim.py $1 ${start_opus} ${end_opus} ${serial}
    # ps xao pgid,comm | grep python3 | cut -f 3 -d ' ' | xargs kill -9
    # for _pid in $(ps xao pid,comm | grep python3 | cut -f 3 -d ' ')
    # do
	  #   pkill -P ${_pid}
    #   echo ${_pid} is dead
    # done
    # for _pgid in $(ps xao pgid,comm | grep python3 | cut -f 3 -d ' ')
    # do
	  #   kill -SIGTERM -- -${_pgid}
    #   echo ${_pgid} is dead
    # done
    start_opus=$((${start_opus}+${n_process}))
    end_opus=$((${start_opus}+${n_process}-1))
    rm -f ../log/nohup.out
    rm -f ../log/nohup.err
  done
fi
