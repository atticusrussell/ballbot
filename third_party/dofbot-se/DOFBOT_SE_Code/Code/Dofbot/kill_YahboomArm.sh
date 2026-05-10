#! /bin/bash

len1=` ps -ef|grep YahboomArm.py |grep -v grep| wc -l`
echo "Number of processes="$len1

if [ $len1 -eq 0 ] 
then
    echo "Yahboom_Arm.py is not running "
else
    # ps -ef| grep YahboomArm.py| grep -v grep| awk '{print $2}'| xargs kill -9 #xargs 将前面的结果作为参数给 kill-9
    
    camera_pid=` ps -ef| grep YahboomArm.py| grep -v grep| awk '{print $2}'`
    kill -9 $camera_pid
    echo "Yahboom_Arm.py is running, and kill it:"
    echo $camera_pid
fi
sleep 1
