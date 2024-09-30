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

secs=$((10))

if [ -x $CHECK_FILE ]; then
	echo "#####################################"
	echo "########## REMOVE UPGRADE FILES #####"
	echo "#####################################"
	rm -rf /opt/SIB.bin
	pkill Upgrade
	echo "#####################################"
	echo "#########POWER-OFF TRIGGERED#########"
	echo "#####################################"
	
	export DISPLAY=:0
	killall -9 matchbox-window-manager

	while [ $secs -gt 0 ]; do
	   echo checking for Upgarde complete "$secs"
	   sleep 1
	   secs=$((secs-1))
	done

	/usr/bin/Xorg &

	export DISPLAY=:0

	./Upgrade_complete &
	
	sleep 1000000
fi

echo -ne '#####                     (33%)\r'
sleep 1
echo -ne '#############             (66%)\r'
sleep 1
echo -ne '#######################   (100%)\r'
echo -ne '\n'
