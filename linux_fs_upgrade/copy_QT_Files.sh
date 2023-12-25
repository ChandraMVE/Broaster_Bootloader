#!/bin/sh
echo "#####################################"
echo "        SCRIPT TO UPGPRADE QT        "
echo "#####################################"

pkill VTC3000QT

mv /opt/FAST/bin/fryApp/vc3000 /opt/FAST/bin/fryApp/backup_vc3000
APP_DIR='/home/root/VTC3000QT'

echo "mount the pendrive"
usbdev='/dev/sda1'
PenDriveMountPath='/home/root/PenDriveMount'
mkdir -p $PenDriveMountPath
mount $usbdev $PenDriveMountPath

CHECK_FILE='/home/root/PenDriveMount/CheckMe.txt'
if [ -x $CHECK_FILE ]; then

	echo "#####################################"
	echo "##########NORMAL BOOT USB UPGARDE#####"
	echo "#####################################"
	
	rm -rf $APP_DIR
	mkdir -p $APP_DIR

	sleep 2
	cp -r $PenDriveMountPath/* $APP_DIR/
	sleep 4
	rm -rf /home/root/PenDriveMount/CheckMe.txt
	sync
	sleep 1
	cp $PenDriveMountPath/VTC3000QT/VTC3000QT $APP_DIR/
	cp $PenDriveMountPath/VTC3000QT_update.sh $APP_DIR/
	cp $APP_DIR/SIB.hex /opt/
	cp $APP_DIR/SIB.bin /opt/
	cp $APP_DIR/SIB_FW_Upgrade.o /opt/
	cp $APP_DIR/auto_SIB_Boot.sh /opt/
	cp $APP_DIR/copy_QT_Files.sh /opt/
	cp $APP_DIR/VTC3000QT_update.sh /opt/
	cp $APP_DIR/mve.service /etc/init.d/
	sync

	chmod 777 $APP_DIR/VTC3000QT
	chmod 777 $APP_DIR/*
	chmod 777 $APP_DIR/VTC3000QT_update.sh
	chmod 777 /etc/init.d/mve.service

	umount $PenDriveMountPath
	umount /dev/sda*
	rm -rf $PenDriveMountPath

	cp $APP_DIR/VTC3000QT_update.sh /opt/
	chmod 777 /opt/VTC3000QT_update.sh
	chmod 777 /opt/SIB_FW_Upgrade.o
	chmod 777 /opt/auto_SIB_Boot.sh
	chmod 777 /opt/copy_QT_Files.sh
	ln -s /etc/init.d/mve.service /etc/rc5.d/S99x96mve.sh
	chmod 777 /etc/rc5.d/S99x96mve.sh

else
	echo "#####################################"
	echo "##########NORMAL BOOT NO UPGARDE#####"
	echo "#####################################"
fi

umount $PenDriveMountPath
rm -rf $PenDriveMountPath

echo "#####################################"
echo "            Upgrade Completed        "
echo "#####################################"

secs=$((10))
while [ $secs -gt 0 ]; do
   echo System reboots in "$secs"
   sleep 1
   secs=$((secs-1))
done

echo "#####################################"
echo "            Please POWER CYCLE       "
echo "#####################################"
