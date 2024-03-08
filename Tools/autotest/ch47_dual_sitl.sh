#!/bin/sh

./fg_ch47_view.sh |
./sim_vehicle.py -v ArduCopter -f heli --model flightgear2 -I 0 --use-dir=dir0 --console --add-param-file=can0.parm |
./sim_vehicle.py -v ArduCopter -f heli --model flightgear2 -I 1 --use-dir=dir1 --console --add-param-file=can1.parm |
dronecan_gui_tool

