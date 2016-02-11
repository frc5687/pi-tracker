v4l2-ctl --set-ctrl=exposure_auto=1
v4l2-ctl --set-ctrl=exposure_absolute=25
java -Djava.library.path=/home/pi/opencv/build/lib/ -jar pi-tracker.jar logging=off images=on
