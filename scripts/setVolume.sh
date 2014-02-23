#!/bin/bash

if [ $1 = "-h" -o $1 = "--help" ]; then
  echo "unmutes computer and sets volume. Enter [0-100] to set volume.  If left blank/given invalid input, defaults too 100"
  exit
fi

number=$(echo $1 | sed "s/[^0-9]//g")

if [ -z $number ]; then
   volume=100
else
  volume=$(echo "scale=0; $number*0.74"| bc) 
fi

amixer set Master playback unmute
sleep 0.1
amixer sset Master,0 $volume
sleep 0.1
amixer sset Master,0 $volume
sleep 0.1

