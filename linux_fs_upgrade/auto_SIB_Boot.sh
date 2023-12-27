#!/bin/bash

CHECK_FILE='/opt/SIB.bin'

secs=$((10))

cd /opt/

export DISPLAY=:0

if [ -x $CHECK_FILE ]; then
killall -9 matchbox-window-manager

while [ $secs -gt 0 ]; do
   echo Upgarde starts in "$secs"
   sleep 1
   secs=$((secs-1))
done

/usr/bin/Xorg &

export DISPLAY=:0

./Upgrade &
fi

./SIB_FW_Upgrade.o

sleep 1


if [ -x $CHECK_FILE ]; then
	echo "#####################################"
	echo "########## REMOVE UPGRADE FILES #####"
	echo "#####################################"
	rm -rf /opt/SIB.bin
	rm -rf /opt/SIB.hex
	pkill Upgrade
	echo "#####################################"
	echo "#########POWER-OFF TRIGGERED#########"
	echo "#####################################"
	poweroff
fi

echo -ne '#####                     (33%)\r'
sleep 1
echo -ne '#############             (66%)\r'
sleep 1
echo -ne '#######################   (100%)\r'
echo -ne '\n'
