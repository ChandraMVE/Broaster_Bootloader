#!/bin/bash

cd /opt/
./SIB_FW_Upgrade.o

sleep 1

CHECK_FILE='/opt/SIB.bin'
if [ -x $CHECK_FILE ]; then
	echo "#####################################"
	echo "########## REMOVE UPGRADE FILES #####"
	echo "#####################################"
	rm -rf /opt/SIB.bin
	rm -rf /opt/SIB.hex
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
