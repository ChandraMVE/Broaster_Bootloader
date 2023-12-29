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
CHECK_FILE_FS='/home/root/PenDriveMount/copy_QT_Files.sh'
if [ -x $CHECK_FILE ]; then

	echo "#####################################"
	echo "##########NORMAL BOOT USB UPGARDE####"
	echo "#####################################"
	
	echo 0 > /etc/rotation
	rm -rf $APP_DIR
	mkdir -p $APP_DIR

	sleep 2	
	if [ -x $CHECK_FILE_FS ]; then
		echo "#####################################"
		echo "##########CRITICAL FS UPGRADE########"
		echo "#####################################"
		cp -r $PenDriveMountPath/copy_QT_Files.sh $APP_DIR/
		cp -r $PenDriveMountPath/mve.service $APP_DIR/
		ln -s /etc/init.d/mve.service /etc/rc5.d/S99x96mve.sh
		chmod 777 /opt/copy_QT_Files.sh
		cp $APP_DIR/copy_QT_Files.sh /opt/
		cp $APP_DIR/mve.service /etc/init.d/
		chmod 777 /etc/init.d/mve.service
		rm -rf $PenDriveMountPath/copy_QT_Files.sh
		rm -rf $PenDriveMountPath/mve.service
		umount $PenDriveMountPath
		umount /dev/sda*
		rm -rf $PenDriveMountPath
		echo "#####################################"
		echo "#############--REBOOT--##############"
		echo "#####################################"
		reboot
	fi
	cp -r $PenDriveMountPath/auto_SIB_Boot.sh $APP_DIR/
	cp -r $PenDriveMountPath/SIB.bin $APP_DIR/
	cp -r $PenDriveMountPath/SIB_FW_Upgrade.o $APP_DIR/
	cp -r $PenDriveMountPath/Upgrade $APP_DIR/
	cp -r $PenDriveMountPath/Upgrade_complete $APP_DIR/
	cp $PenDriveMountPath/VTC3000QT_update.sh $APP_DIR/
	
	rm -rf /home/root/PenDriveMount/CheckMe.txt
	sync
	cp -r $PenDriveMountPath/VTC3000QT $APP_DIR/	
	sleep 1
	sync
	cp $APP_DIR/SIB.bin /opt/
	cp $APP_DIR/SIB_FW_Upgrade.o /opt/
	cp $APP_DIR/auto_SIB_Boot.sh /opt/
	cp $APP_DIR/VTC3000QT_update.sh /opt/	
	cp $APP_DIR/Upgrade /opt/
	cp $APP_DIR/Upgrade_complete /opt/
	sync

	chmod 777 $APP_DIR/VTC3000QT
	chmod 777 $APP_DIR/*
	chmod 777 $APP_DIR/VTC3000QT_update.sh

	umount $PenDriveMountPath
	umount /dev/sda*
	rm -rf $PenDriveMountPath

	cp $APP_DIR/VTC3000QT_update.sh /opt/
	chmod 777 /opt/VTC3000QT_update.sh
	chmod 777 /opt/SIB_FW_Upgrade.o
	chmod 777 /opt/auto_SIB_Boot.sh
	chmod 777 /opt/Upgrade
	chmod 777 /opt/Upgrade_complete
	chmod 777 /etc/rc5.d/S99x96mve.sh

else
	echo "#####################################"
	echo "##########NORMAL BOOT NO UPGARDE#####"
	echo "#####################################"
fi

umount $PenDriveMountPath
rm -rf $PenDriveMountPath

secs=$((10))
while [ $secs -gt 0 ]; do
   sleep 1
   secs=$((secs-1))
done
