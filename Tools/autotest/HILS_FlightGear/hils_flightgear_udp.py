# This Python file uses the following encoding: utf-8
#HILS for run ./fg_ch47_view.sh
# MAVProxy    mavproxy.py --master=/dev/ttyACM0 --console --map
#virtual serial port  socat -d -d pty,raw,echo=0 pty,raw,echo=0
#usage:
#PARAMS:
# param set AHRS_EKF_TYPE 11
# param set EAHRS_TYPE 3
# param set EAHRS_RATE 50
# param set SERIAL2_PROTOCOL 36  
# param set SERIAL2_BAUD 460 460800
# param set GPS_TYPE 21 <--GPS_TYPE_EXTERNAL_AHRS = 21,
# param set FS_OPTIONS 0  <--Failsafe disable
# param set DISARM_DELAY 0
# param set ARMING_CHECk 0
# param set H_SW_TYPE 1
# param set BRD_SAFETY_DEFLT 0
# param set ATC_RATE_Y_MAX 10
# param set INS_USE2 0
# param set INS_ENABLE_MASK 1
# param set INS_FAST_SAMPLE 1
# param set RK3_SRC1_POSZ 3
# param set ATC_HOVR_ROL_TRM 0
# param set CAN_SLCAN_CPORT 1
# RC_OPTION ignore rc receiver
import sys
import os
import sys
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *
from PyQt5 import uic
import serial
import serial.tools.list_ports
import FGcommon
import uavcan_common

# python -m PyQt5.uic.pyuic -x mainwindow.ui -o mainwindow.py


RAD2DEG = 57.2958

class HILS_FlightGear(QtWidgets.QDialog):
    
    def __init__(self, parent = None):
       super().__init__(parent)
       self.ui = uic.loadUi("form.ui", self)
       self.ui.show()

       
       self.init_serial_ports()

       text, port_list = FGcommon.get_pixhawk_port()
       self.ui.textEdit_4.setText(text)
       for i in range(len(port_list)):
         self.ui.comboBox.addItem(port_list[i])

       self.udp = FGcommon.udp_socket("127.0.0.1:9003:9002")
       self.udp1 = FGcommon.udp_socket("127.0.0.1:9012:9013")
       #self.udp2 = FGcommon.udp_socket("127.0.0.1:9022:9023")
       self.pushButton_1.clicked.connect(self.handelButton_1)
       self.pushButton_2.clicked.connect(self.handelButton_2)
       self.pushButton_3.clicked.connect(self.handelButton_3)
       self.pushButton_4.clicked.connect(self.handelButton_4)
       #self.pushButton_5.clicked.connect(self.handelButton_5)
       self.udp.intReady.connect(self.update_textedit)
       #self.udp.packReady.connect(self.send_data_to_serial)
       self.udp.packReady.connect(self.send_data_to_gcs_udp)
       #self.udp2.intReady.connect(self.update_textedit)
       #self.udp2.packReady.connect(self.send_data_to_udp)
       self.last_timestamp = 0;
    
   
       
    def init_serial_ports(self):
       self.port1 = None
       self.port2 = None
       self.serial1 = None
       self.serial2 = None
       port1 , hwid1, port2, hwid2 = FGcommon.get_serial_port()
       
       self.port1 = port1
       self.port2 = port2
       if self.port1 != None:
         self.ui.label.setText(port1 + hwid1)
         self.serial1 = FGcommon.usbSerial(port1)
         self.serial1.packReady.connect(self.receive_from_serial1)
       if self.port2 != None:
         self.ui.label_2.setText(port2 + hwid2)
         self.serial2 = FGcommon.usbSerial(port2)
         self.serial2.packReady.connect(self.receive_from_serial2)  

    def closeEvent(self, event):
        self.udp.close()
        
        if self.serial1 != None:
            self.serial1.close()
            print("serial1 closed\n")
        if self.serial2 != None:
            self.serial2.close()
            print("serial1 closed\n")
         

    def handelButton_5(self):
       port = self.ui.comboBox.currentText()
       self.can = uavcan_common.uavcan(port)
       self.can.msgReady.connect(self.update_textEdit_5)
       
      
      
    def update_textEdit_5(self, msg):
       self.ui.textEdit_5.setText(msg)
       
    def handelButton_1(self):

       text, port_list = FGcommon.get_pixhawk_port()
       self.ui.textEdit_4.setText(text)
       self.ui.comboBox.clear()
       for i in range(len(port_list)):
         self.ui.comboBox.addItem(port_list[i])
                   
    def receive_from_serial1(self, data):
        unpack, pack = self.serial1.parsing_data_from_serial(data)

        #self.udp.sender(pack)
        #print(pack)
        if unpack != None:
            text = ''
            for i in range(len(unpack)) :
                  text += 'ch['+ str(i) + ']' + str(unpack[i])+'\n'
            
            self.ui.textEdit_2.setText(text)
            self.update_servo_output_slider1(unpack)
            self.udp.write(pack)
            self.handelButton_2()
   
    def receive_from_serial2(self, data):
        unpack, pack = self.serial2.parsing_data_from_serial(data)

        #self.udp.sender(pack)
        #print(pack)
        if unpack != None:
            text = ''
            for i in range(len(unpack)) :
                  text += 'ch['+ str(i) + ']' + str(unpack[i])+'\n'
            
            self.ui.textEdit_3.setText(text)
            self.update_servo_output_slider2(unpack)
            self.udp.write(pack)
            self.handelButton_3()


    def handelButton_2(self):
       self.ui.pushButton_2.setIcon(self.style().standardIcon(getattr(QStyle, 'SP_DialogApplyButton')))
       self.ui.pushButton_3.setIcon(self.style().standardIcon(getattr(QStyle, 'SP_DialogCancelButton')))
    
    def handelButton_3(self):
       self.ui.pushButton_3.setIcon(self.style().standardIcon(getattr(QStyle, 'SP_DialogApplyButton')))
       self.ui.pushButton_2.setIcon(self.style().standardIcon(getattr(QStyle, 'SP_DialogCancelButton')))
    
    def handelButton_4(self):
       
       port = self.ui.comboBox.currentText()
       FGcommon.set_param(port)
       self.ui.textEdit_4.setText('Param Set via port='+port)
       
    def open_serial_port(self):
       pass
    
    def send_data_to_serial(self, data):
       
       if self.serial1 != None:
          self.serial1.send(data)
       if self.serial2 != None:
          self.serial2.send(data)

    def send_data_to_gcs_udp(self, data):
       
       if self.udp1 != None:
          self.udp1.write(data)
          print(data)
      # if self.udp2 != None:
      #    self.udp2.write(data)
      #    print(data)   

          
    def update_textedit(self, upack_data):
      
      dT = upack_data[0] - self.last_timestamp
      self.last_timestamp = upack_data[0]
      data = ''
      data += 'dT(ms) : {:.1f}\n '.format(dT)
      data += 'GPSt(sec) : {:.3f}\n'.format(upack_data[0]*1e-3)
      data += 'Latitude : {:.7f}\n'.format(upack_data[1])
      data += 'Longitude: {:.7f}\n'.format(upack_data[2])
      data += 'Hgt(m)   : {:.3f}\n'.format(upack_data[3])
      data += 'p(deg/s) : {:.3f}\n'.format(upack_data[4]*RAD2DEG)
      data += 'q(deg/s) : {:.3f}\n'.format(upack_data[5]*RAD2DEG)
      data += 'r(deg/s) : {:.3f}\n'.format(upack_data[6]*RAD2DEG)
      data += 'Acc_x(N) : {:.3f}\n'.format(upack_data[7])
      data += 'Acc_y(N) : {:.3f}\n'.format(upack_data[8])
      data += 'Acc_z(N) : {:.3f}\n'.format(upack_data[9])
      data += 'Velx(m/s): {:.3f}\n'.format(upack_data[10])
      data += 'Vely(m/s): {:.3f}\n'.format(upack_data[11])
      data += 'Velz(m/s): {:.3f}\n'.format(upack_data[12])
      data += 'Roll(deg): {:.1f}\n'.format(upack_data[13]*RAD2DEG)
      data += 'Pitch(deg): {:.1f}\n'.format(upack_data[14]*RAD2DEG)
      data += 'Yaw(deg) : {:.1f}\n'.format(upack_data[15]*RAD2DEG)
      data += 'baro(psc): {:.1f}\n'.format(upack_data[16])
      data += 'Rotor(rpm): {:.1f}\n'.format(upack_data[17])
      data += 'MegX(gauss): {:.3f}\n'.format(upack_data[18])
      data += 'MegY(gauss): {:.3f}\n'.format(upack_data[19])
      data += 'MegZ(gauss): {:.3f}'.format(upack_data[20])
      self.ui.textEdit.setText(data)

    def update_servo_output_slider1(self,servo_value):
        self.ui.verticalSlider_1.setValue(servo_value[0])
        self.ui.verticalSlider_2.setValue(servo_value[1])
        self.ui.verticalSlider_3.setValue(servo_value[2])
        self.ui.verticalSlider_4.setValue(servo_value[3])
    
    def update_servo_output_slider2(self,servo_value):
        self.ui.verticalSlider_5.setValue(servo_value[0])
        self.ui.verticalSlider_6.setValue(servo_value[1])
        self.ui.verticalSlider_7.setValue(servo_value[2])
        self.ui.verticalSlider_8.setValue(servo_value[3])

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    me = HILS_FlightGear()
    sys.exit(app.exec_())
