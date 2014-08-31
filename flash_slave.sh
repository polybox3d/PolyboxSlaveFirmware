#!/bin/bash

########################################################################
## 
## Start a process to flash several slave board.
## Do not plug your board until the script ask you
##
##
########################################################################

currentid=1

if test -z $1
then
    echo "give number of board to flash"
    exit
fi
for (( i=1; i<=$1; i++ ))
do
    make clean
    echo "Plug your board via USB now. \nPress a key when ready."
    read
    usb=`ls -t /dev/ |grep ttyAC | head -n 1`
    usb=/dev/$usb
    make SLAVE_ID=$i
    make up DEVICE=$usb
    currentid=$currentid+1
done
