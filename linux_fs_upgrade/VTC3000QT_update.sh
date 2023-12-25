#!/bin/sh

killall -9 matchbox-window-manager

secs=$((10))
while [ $secs -gt 0 ]; do
   echo KB application Starts in "$secs"
   sleep 1
   secs=$((secs-1))
done

export DISPLAY=:0

echo "#########################"
echo "Kitchen Brain Application"
echo "#########################"

cd /home/root/VTC3000QT/VTC3000QT

/usr/bin/Xorg &

export DISPLAY=:0

./VTC3000QT &

exit 0
