#!/usr/bin/env python3
import socket

class WifiConnection(object):
        def __init__(self):
                self.ipaddr = '192.168.3.1'
                self.port = 8080
                self.socket = None
                self.pcClient = None
                self.isConnected = False
                self.reconn_counter = 0

        def connect(self):
                try:
                        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.socket.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
                        self.socket.bind((self.ipaddr,self.port))
                        self.socket.listen(1)
                        print('Waiting for socket connection')
                        self.pcClient, self.pcClientIP = self.socket.accept()
                        print('PC connected from ', self.pcClientIP)
                        self.isConnected = True
                except Exception as e:
                        print ('WifiConnection error: ', str(e))

        def read(self):
                try:
                        #print('Reading')
                        received_data = self.pcClient.recv(1024)
                        received_data = received_data.decode("utf-8")
                        if received_data == '':
                                print('Host Disconnected')
                                self.isConnected = False
                                print('Trying to Reconnect')
                                self.connect()
                        #print("Received [%s] of size [%d]" % (received_data,len(received_data)))
                        return received_data
                except Exception as e:
                        print ("PC Read Error: ", str(e))
                        print('Reconnect in read attempt %d'%(self.reconn_counter))
                        self.reconn_counter += 1
                        self.connect()

        def write(self,strMsg):
                try:
                        if self.isConnected:
                                self.pcClient.sendto(strMsg+'\r\n',self.pcClientIP)
                        else:
                                print('Host is not connected')
                                self.isConnected = False
                                print('Reconnect in write attempt %d'%(self.reconn_counter))
                                self.reconn_counter += 1
                                self.connect()
                except Exception as e:
                        print ("Wifi Write Error: ", str(e))
                        print('Reconnect Attempt')
                        self.reconn_counter += 1
                        self.connect()

        def close(self):
                try:
                        if self.socket:
                                self.socket.close()
                        if self.pcClient:
                                self.pcClient.close()
                        self.isConnected = False
                        print ('Wifi Terminated')
                except Exception as e:
                        print('Error in Closing Wifi: ', str(e))

if __name__ == "__main__" :
        print('In WifiConnection')
        wifi = WifiConnection()
        wifi.__init__()
        wifi.connect()
        counter = 0
        #wifi.read()
        
        while True:
                try:
                        if wifi.isConnected:
                                #if data available in buffer
                                #wifi.write('RPI sends: '+str(counter))
                                #counter += 1
                                #print('test')
                                str = raw_input()
                                print("Write: %s " % str)
                                wifi.write(str)
                                recv_text = wifi.read()
                                print(recv_text)
                except KeyboardInterrupt:
                        print('WifiConnection interrupted')
                        break
                        
        wifi.close()
