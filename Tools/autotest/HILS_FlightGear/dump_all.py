#!/usr/bin/env python3
'''
dump all messages in YAML format
'''

import dronecan, time, math
import yaml

# get command line arguments
from argparse import ArgumentParser
parser = ArgumentParser(description='dump all DroneCAN messages')
parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
parser.add_argument("--dna-server", action='store_true', default=False, help="run DNA server")
parser.add_argument("port", default=None, type=str, help="serial port")
args = parser.parse_args()
    
# Initializing a DroneCAN node instance.
node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

# Initializing a node monitor, so we can see what nodes are online
node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

def handle_notify_state(msg):
    print(dronecan.to_yaml(msg))
    msg2 = yaml.load(dronecan.to_yaml(msg))
    vehicl_state = msg2["vehicle_state"]
    print(bin(vehicl_state))
    a=''
    if vehicl_state & (1<<0) > 0 :
        a +='VEHICLE_STATE_INITIALISING | '
    if vehicl_state & (1<<1) > 0 :
        a +='VEHICLE_STATE_ARMED | '
    if vehicl_state & (1<<2) > 0 :
        a += 'VEHICLE_STATE_FLYING | '
    if vehicl_state & (1<<3) > 0 :
        a += 'VEHICLE_STATE_PREARM | '
    if vehicl_state & (1<<4) > 0 :
        a += 'VEHICLE_STATE_PREARM_GPS | '
    if vehicl_state & (1<<5) > 0 :
        a += 'VEHICLE_STATE_SAVE_TRIM | '
    if vehicl_state & (1<<6) > 0 :
        a += 'VEHICLE_STATE_ESC_CALIBRATION | '
    if vehicl_state & (1<<7) > 0 :
        a +='VEHICLE_STATE_FAILSAFE_RADIO | '
    if vehicl_state & (1<<8) > 0 :
        a +='VEHICLE_STATE_FAILSAFE_BATT | '
    if vehicl_state & (1<<9) > 0 :
        a += 'VEHICLE_STATE_FAILSAFE_GCS | '
    if vehicl_state & (1<<10) > 0 :
        a += 'VEHICLE_STATE_CHUTE_RELEASED | '
    if vehicl_state & (1<<11) > 0 :
        a += 'VEHICLE_STATE_EKF_BAD | '
    if vehicl_state & (1<<12) > 0 :
        a += 'VEHICLE_STATE_FW_UPDATE | '
    if vehicl_state & (1<<13) > 0 :
        a += 'VEHICLE_STATE_MAGCAL_RUN | '
    if vehicl_state & (1<<14) > 0 :
        a += 'VEHICLE_STATE_LEAK_DET | '
    if vehicl_state & (1<<15) > 0 :
        a += 'VEHICLE_STATE_GPS_FUSION | '
    if vehicl_state & (1<<16) > 0 :
        a += 'VEHICLE_STATE_GPS_GLITCH | '
    if vehicl_state & (1<<17) > 0 :
        a +='VEHICLE_STATE_POS_ABS_AVAIL | '
    if vehicl_state & (1<<18) > 0 :
        a +='VEHICLE_STATE_LOST | '
    if vehicl_state & (1<<19) > 0 :
        a += 'VEHICLE_STATE_THROW_READY | '
    if vehicl_state & (1<<20) > 0 :
        a += 'VEHICLE_STATE_POWERING_OFF | '
    if vehicl_state & (1<<21) > 0 :
        a += 'VEHICLE_STATE_VIDEO_RECORDING | '
    if vehicl_state & (1<<22) > 0 :
        a += 'VEHICLE_STATE_IS_LANDING | '
    if vehicl_state & (1<<23) > 0 :
        a += 'VEHICLE_STATE_IS_TAKING_OFF | '


    print(a)


if args.dna_server:
    # optionally start a DNA server
    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

# callback for printing all messages in human-readable YAML format.
#node.add_handler(None, lambda msg: print(dronecan.to_yaml(msg)))
node.add_handler(dronecan.ardupilot.indication.NotifyState, handle_notify_state)

# Running the node until the application is terminated or until first error.
try:
    node.spin()
except KeyboardInterrupt:
    pass
