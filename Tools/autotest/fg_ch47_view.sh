#!/bin/sh
#https://wiki.flightgear.org/Property_Tree/Native_Protocol_Slaving
#./sim_vehicle.py -v ArduCopter -f heli -L PHNL --model flightgear2:127.0.0.1 --map --console
#    --generic=socket,in,10,,5003,udp,MAVLink \
#    --generic=socket,out,10,,5001,udp,MAVLink \
#    { "AHRS_EKF_TYPE", 10 },
#    { "INS_GYR_CAL", 0 },
AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --generic=socket,in,20,,9002,udp,MAVLink \
    --generic=socket,out,20,,9003,udp,MAVLink \
    --generic=socket,in,20,,9012,udp,MAVLink \
    --generic=socket,out,20,,9013,udp,MAVLink \
    --model-hz=120 \
    --aircraft=ch47 \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=PHNL \
    --geometry=650x550 \
    --bpp=32 \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --disable-anti-alias-hud \
    --wind=0@0 \
    $*