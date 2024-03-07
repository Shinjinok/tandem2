# This Python file uses the following encoding: utf-8

# if__name__ == "__main__":
#     pass

import serial
import socket
from struct import pack, unpack
import threading
import time
from PyQt5.QtCore import QObject, QThread, pyqtSignal, pyqtSlot
from pymavlink import mavextra
import dronekit
Param = []
Param.append(['AHRS_EKF_TYPE',11])
Param.append(['EAHRS_TYPE', 3])
Param.append(['EAHRS_RATE',1000]) #<-set 1000 for avoiding critical error 
Param.append(['SERIAL2_PROTOCOL', 36])  
Param.append(['SERIAL2_BAUD', 460]) #460800
Param.append(['GPS_TYPE', 21]) #<--GPS_TYPE_EXTERNAL_AHRS = 21,
Param.append(['FS_OPTIONS', 0]) 
Param.append(['DISARM_DELAY', 0])
Param.append(['ARMING_CHECK', 0])
Param.append(['H_SW_TYPE', 1])
Param.append(['BRD_SAFETY_DEFLT', 0])
Param.append(['ATC_RATE_Y_MAX', 10])
Param.append(['INS_USE2', 0])
Param.append(['INS_USE3', 0])
Param.append(['INS_ENABLE_MASK', 3]) #<-set 3 for avoiding low loop late
Param.append(['EK3_IMU_MASK', 1])
Param.append(['H_COL_ANG_MIN', -5])
Param.append(['H_COL_ANG_MAX', 16])
Param.append(['RC_OPTIONS', 1])
Param.append(['ATC_HOVR_ROL_TRM',0])
Param.append(['CAN_SLCAN_CPORT', 0])
Param.append(['CAN_P1_DRIVER', 1])

Param.append(['ATC_ANG_PIT_P', 1.0])
Param.append(['ATC_ANG_RLL_P', 1.0])
Param.append(['ATC_ANG_YAW_P', 1.0])

Param.append(['ATC_RAT_PIT_D', 0.5])
Param.append(['ATC_RAT_PIT_I', 0.01])
Param.append(['ATC_RAT_PIT_P', 1.0])

Param.append(['ATC_RAT_RLL_D', 0.5])
Param.append(['ATC_RAT_RLL_I', 0.01])
Param.append(['ATC_RAT_RLL_P', 1.0])

Param.append(['ATC_RAT_YAW_D', 0.5])
Param.append(['ATC_RAT_YAW_I', 0.01])
Param.append(['ATC_RAT_YAW_P', 1.0])

Param.append(['PSC_VELXY_D', 0.25])
Param.append(['PSC_VELXY_P', 0.5])
Param.append(['PSC_VELXY_I', 0.01])

Param.append(['PSC_VELZ_D', 0.5])
Param.append(['PSC_VELZ_P', 1.0])
Param.append(['PSC_VELZ_I', 0.01])

Param.append(['ATC_RATE_FF_ENAB', 0])
Param.append(['PSC_POSXY_P', 2.0])

Param.append(['EK3_SRC1_POSXY', 3])
Param.append(['EK3_SRC1_POSZ', 3])
Param.append(['EK3_SRC1_VELXY', 3])
Param.append(['EK3_SRC1_VELZ', 3])

Param.append(['FLTMODE1', 5])
Param.append(['FLTMODE6', 1])


# RC_OPTION ignore rc receiver
#1/1/1970~1/6/1980 522weeks 3days tokyo utc+9 and correct 18secs
FIX_TIME = -522*7*24*60*60*1000 - 3*24*60*60*1000  + 9*60*60*1000 + 18*1000
FEET2METER = 0.3048
DEG2RAD = 0.0174533

class GPS:
  lat = None
  lon = None
  fix_type = 5
class ATT:
  roll = None
  pitch = None
  yaw = None

port =''

def set_param(set_port):
   global port
   port = set_port
   t = threading.Thread(target=set_param_thread)
   t.start()

def set_param_thread():
  global port
  get = 'Parammeter Set:\n'
  
  try:
    vehicle = dronekit.connect(port, wait_ready=True, baud=57600)
    for i in range(len(Param)):
      vehicle.parameters[Param[i][0]] = Param[i][1]
      get += Param[i][0] + '=' + str(Param[i][1]) +'\n'
      text_set = 'Pixhawk Parameter setting .. [' + str(i+1) + '/' +str(len(Param))+']'
      print(text_set)
    vehicle.reboot()
    print('reboot pixhawk\n')
    vehicle.close()
    print('Mavlink link close\n')
  except:
     print(port + 'Paramerter Setup Fail, Try change port')


   

def get_pixhawk_port():
   ports = serial.tools.list_ports.comports()
 
   text = ''
   port_list=[]
   ser = 'Pix'
   for port, desc, hwid in sorted(ports):
      try:
        desc.index(ser)
        text += "{}: {} [{}]\n".format(port, desc, hwid)
        port_list.append(port)
      except:
        pass
   return text, port_list

def get_serial_port():
    port1 = None
    port2 = None
    hwid1 = None
    hwid2 = None
    ports = serial.tools.list_ports.comports()

    for port, desc, hwid in sorted(ports):
      try:
        ser = 'SER=0001'
        hwid.index(ser)
        port1 = port
        hwid1 = '['+ ser+']'
        print("{}: {} [{}]".format(port, desc, hwid))
        print("port1=",port1)
      except:
        print("Not found")
      try:
        ser = 'SER=0002'
        hwid.index(ser)
        hwid2 = '['+ ser+']'
        port2 = port
        print("{}: {} [{}]".format(port, desc, hwid))
        print("port2=",port2)
      except:
        print("Not found")

    return port1, hwid1, port2, hwid2

class udp_socket(QObject):
  """A UDP socket."""
  finished = pyqtSignal()
  intReady = pyqtSignal(list)
  packReady = pyqtSignal(bytes)
  
  
  def __init__(self, device):
    super(udp_socket, self).__init__()
    self.working = True
    a = device.split(':')
    print(a)

    self.port = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    self.port.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    self.port.settimeout(1)
    self.port.bind((a[0], int(a[1])))
    print("bind port",a[1])
    self.destination_addr = (a[0], int(a[2]))
    self.udp_data_in = None
    self.udp_data_in_flag = False
    self.udp_raw_data_in = None
    

    t = threading.Thread(target=self.listen_clients)
    self.thread_run = True
    t.start()

  def write(self, buf):
    try:
        self.port.sendto(buf, self.destination_addr)
        print(self.destination_addr)

    except socket.error:
        pass

  def close(self):
     self.thread_run = False
     print("udp close")

  def listen_clients(self):
    print(self.destination_addr, " start thread\n")
    time.sleep(1)
    while self.thread_run:
        try:
          data_udp ,addr= self.port.recvfrom(1024)
        except socket.timeout:
          continue  
        if(len(data_udp) > 0):
          unpack_data = self.unpacking_data(data_udp)
          d = self.packing_data(unpack_data)
          self.packReady.emit(d)
          self.intReady.emit(unpack_data)
          #self.packReady.emit(data_udp)

          time.sleep(0.001)
    
    print('udp thread stop\n')

  def packing_data(self, data):
    d = pack('<3B3d18f',0xFE, 0xBB, 0xAA, 
             data[0],data[1],data[2],data[3],data[4],data[5],
             data[6],data[7],data[8],data[9],data[10],data[11],data[12],
             data[13],data[14],data[15],data[16],data[17],data[18],
             data[19],data[20])
    return d
    
    
  
  def unpacking_data(self, data_udp):
    
    udp_data_in = unpack('>3d15f',data_udp)

    gps = GPS()
    att = ATT()
    gps.lat = udp_data_in[1] *1e7
    gps.lon = udp_data_in[2] *1e7
    att.roll = udp_data_in[13]*DEG2RAD
    att.pitch = udp_data_in[14]*DEG2RAD
    att.yaw = udp_data_in[15]*DEG2RAD
    #hgt = udp_data_in[3] * 100
    m = mavextra.expected_mag(gps, att)
    baro = float(udp_data_in[16]*3386.39) # Convert millibar to pascals
    gps_time= time.time()*1000 + FIX_TIME #msec

    SendDataList = list()
    SendDataList.append(gps_time)
    SendDataList.append(udp_data_in[1])
    SendDataList.append(udp_data_in[2])
    SendDataList.append(udp_data_in[3])
    SendDataList.append(udp_data_in[4])
    SendDataList.append(udp_data_in[5])
    SendDataList.append(udp_data_in[6])
    SendDataList.append(udp_data_in[7]*FEET2METER)
    SendDataList.append(udp_data_in[8]*FEET2METER)
    SendDataList.append(udp_data_in[9]*FEET2METER)
    SendDataList.append(udp_data_in[10]*FEET2METER)
    SendDataList.append(udp_data_in[11]*FEET2METER)
    SendDataList.append(udp_data_in[12]*FEET2METER)
    SendDataList.append(udp_data_in[13]*DEG2RAD)
    SendDataList.append(udp_data_in[14]*DEG2RAD)
    SendDataList.append(udp_data_in[15]*DEG2RAD)
    SendDataList.append(baro)
    SendDataList.append(udp_data_in[17])
    SendDataList.append(m.x)
    SendDataList.append(m.y)
    SendDataList.append(m.z)

    return SendDataList


class usbSerial(QObject):
    packReady = pyqtSignal(bytes)

    def __init__(self, port):
        super(usbSerial, self).__init__()
        self.port = port
        self.serial = None
        try:
          self.serial = serial.Serial(port, baudrate=460800, timeout=1)
        except Exception as e:
          print("error open serial port: " + str(e))
          exit()
        
        #self.lock = threading.Lock()
        
        t = threading.Thread(target=self.listen)
        self.thread_run = True
        t.start()

    @pyqtSlot(bytes)
    def listen(self):
        print(self.port, " start thread\n")
        #time.sleep(1)

        while self.thread_run :
          time.sleep(0.001)
          serial_data_in = self.serial.readline()
          if serial_data_in != None:
              self.packReady.emit(serial_data_in)

        #time.sleep(1)
        self.serial.cancel_write()
        self.serial.close()      
        print("serial" ,self.port, " thread stop\n")
        
    
    def send(self,in_data):
        if self.serial.writable():
          self.serial.write(in_data)

    def parsing_data_from_serial(self, serial_data_in):
        a = str(serial_data_in).split(':')
        print(a)
        if len(a) == 10:
          rcv_data = []
          for i in range(4): rcv_data.append(int(a[i+1]))
          send_data = []
          send_data.append(float((float(a[1] ) - 1500.0) / 250.0))
          send_data.append(float((float(a[2] ) - 1500.0) / -250.0))
          send_data.append(float((2000.0 - float(a[3] )) / 1000.0))
          send_data.append(float((float(a[4] ) - 1500.0) / 500.0))
          send_pack= pack('>5f',send_data[0],send_data[1] ,send_data[2] ,send_data[3] ,send_data[2])
        else:
          return None, None
        
        return rcv_data, send_pack
    
    def close(self):         
      self.thread_run = False







