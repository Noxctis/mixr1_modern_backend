# -*- coding: utf-8 -*-

import serial
import serial.tools.list_ports as list_ports
import time
import threading


class Comm(object):
    def __init__(self):
        self.isOpen = False
        self.isReadFail = 0
        self.port = None

    def open(self, comport='COM4', rate=15200):
        try:
            self.port = serial.Serial(port=comport, baudrate=rate, timeout=0.1)
            if self.port.isOpen():
                self.port.close()
            self.port.open()
            self.isOpen = True
        except serial.SerialException:
            self.isOpen = False

    def send_cmd(self, cmd):
        ret_code = ''
        try:
            for i in range(len(cmd)):
               # print('cmd',cmd[i])
                n=self.port.write(cmd[i].encode())
                #print(n)
                ret_code = self.port.read(n)
                #self.port.write(cmd[i])
                #ret_code += self.port.read(1)
        except serial.portNotOpenError:
            ret_code = 'ERROR:send_cmd\n'
        return ret_code

    def rec_msg(self):
        ret_code = ''
        try:
            n = self.port.inWaiting()
            if n != 0:
                ret_code = bytes.decode(self.port.read(n))
        except serial.portNotOpenError:
            ret_code = 'ERROR:rec_msg\n'
        return ret_code

    def close(self):
        self.port.close()
        self.isOpen = False


class SerialCOM(object):
    def __init__(self, com_name):
        self.comm = Comm()
        self.lComName = []
        self.threadStatus = 0  # 0:stop 1:start 2:pause
        self.rec_thread = None
        self.callback = None
        self.msg = None
        self.logfile = None
        port_list = list(list_ports.grep(com_name))
        if len(port_list) > 0:
            for iPort in range(0, len(port_list)):
                port = port_list[iPort]
                self.lComName.append(port[0])

    def __del__(self):
        self.disconnect()

    def connect(self, com,rate, callback):
        if not self.comm.isOpen:
            self.comm.open(com, rate)
        if self.comm.isOpen and self.threadStatus == 0:
            self.callback = callback
            self.logfile = open('com_log.txt', 'w')
            self.rec_thread = threading.Thread(target=self.com_thread)
            self.threadStatus = 1
            self.rec_thread.start()
        return self.comm.isOpen

    def disconnect(self):
        if self.threadStatus:
            self.threadStatus = 0
            self.rec_thread.join()
        if self.comm.isOpen:
            self.comm.close()
            self.logfile.close()
        return not self.comm.isOpen

    def send_command(self, cmd):
        #print('COMMUNICATION',cmd)
        self.threadStatus = 2
        time.sleep(0.1)
        self.comm.send_cmd(cmd)
        time.sleep(0.1)
        self.threadStatus = 1

    def com_thread(self):
        while self.threadStatus:
            while self.comm.isOpen and self.threadStatus == 1:
                self.msg = self.comm.rec_msg()
                if len(self.msg) > 0:
                    self.logfile.write(self.msg)
                    self.callback(self)
                time.sleep(0.1)
            time.sleep(0.1)
