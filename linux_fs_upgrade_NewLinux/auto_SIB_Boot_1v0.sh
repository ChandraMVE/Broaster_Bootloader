#!/bin/sh

CHECK_FILE='/opt/SIB.bin'

cd /opt/

if [ -x $CHECK_FILE ]; then

	export QT_QPA_PLATFORM=linuxfb:fb=/dev/fb0:size=1024x600:mmSize=1024x600
	./Upgrade &
fi

./SIB_FW_Upgrade.o

sleep 1

if [ -x $CHECK_FILE ]; then
	echo "#####################################"
	echo "########## REMOVE UPGRADE FILES #####"
	echo "#####################################"
	rm -rf /opt/SIB.bin
	kill -9 $(pidof Upgrade)
	

	export QT_QPA_PLATFORM=linuxfb:fb=/dev/fb0:size=1024x600:mmSize=1024x600
	./Upgrade_complete &	
	sleep 100000000
	wait
fi
