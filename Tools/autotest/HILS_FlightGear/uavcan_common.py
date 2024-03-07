#!/usr/bin/env python3
'''
dump all messages in YAML format
'''

import dronecan

from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
# get command line arguments
from argparse import ArgumentParser
import threading
import time

class uavcan(QObject):
    msgReady = pyqtSignal(str)
    
    def __init__(self, port):
        super(uavcan, self).__init__()
        
        parser = ArgumentParser(description='dump all DroneCAN messages')
        parser.add_argument("--bitrate", default=1000000, type=int, help="CAN bit rate")
        parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
        parser.add_argument("--dna-server", action='store_true', default=False, help="run DNA server")
        parser.add_argument("--port", default = port, type=str, help="serial port")
        args = parser.parse_args()
    
        # Initializing a DroneCAN node instance.
        self.node = dronecan.make_node(args.port, node_id=args.node_id, bitrate=args.bitrate)

        # Initializing a node monitor, so we can see what nodes are online
        #node_monitor = dronecan.app.node_monitor.NodeMonitor(self.node)

        #if args.dna_server:
            # optionally start a DNA server
        #    dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.node, node_monitor)

        # callback for printing all messages in human-readable YAML format.
        #self.node.add_handler(None, lambda msg: print(dronecan.to_yaml(msg)))
        self.node.add_handler(None, lambda msg: self.msg_signal(dronecan.to_yaml(msg)))
        
        t = threading.Thread(target=self.node_thread)
        self.node_running = True
        t.start()

    def msg_signal(self, msg):
        self.msgReady.emit(msg)
    def msg_signal(self):
        msg = 'callback'
        self.msgReady.emit(msg)    
        
    def node_thread(self):
        # Running the node until the application is terminated or until first error.
        while self.node_running:
            self.node.spin()
            
        print('uavcan stop thread')
    
    def close(self):
        self.node_running = False