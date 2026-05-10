#! /bin/bash

len1=` ps -ef|grep yahboom_oled.py |grep -v grep| wc -l`
echo "Number of processes="$len1

if [ $len1 -eq 0 ] 
then
    echo "yahboom_oled.py is not running "
else
    # ps -ef| grep yahboom_oled.py| grep -v grep| awk '{print $2}'| xargs kill -9  
    camera_pid=` ps -ef| grep yahboom_oled.py| grep -v grep| awk '{print $2}'`
    kill -9 $camera_pid
    echo "yahboom_oled.py killed, PID:"
    echo $camera_pid
    
    # Clear OLED
    python3 /home/jetson/software/oled_yahboom/yahboom_oled.py clear
fi
sleep .01
