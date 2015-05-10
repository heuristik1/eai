#!/bin/bash

while true; 
do
	pkill -f main.py 
	sleep 1
	(python /home/pi/eai-proj010315/main.py) &
	sleep 6h
done