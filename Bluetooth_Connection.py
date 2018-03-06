import serial
from bluetooth import *

class Bluetooth_Connection(object):
    def __init__(self):
        self.bluetoothPort = 4
        self.bluetoothSocket = None

    def setupConnection(self):
        try:
            #Create socket
            self.bluetoothSocket = BluetoothSocket(RFCOMM)
            print("Socket for bluetooth created")
            
            #Binding socket
            self.bluetoothSocket.bind(("",self.bluetoothPort))
            print("Bluuetooth connection bounded")
            
            #Waiting for device
            self.bluetoothSocket.listen(1)
            port = self.bluetoothSocket.getsockname()[1]
            
            uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"
            advertise_service( self.bluetoothSocket, "MDP_GROUP_3", service_id = uuid, service_classes = [ uuid, SERIAL_PORT_CLASS ], profiles = [ SERIAL_PORT_PROFILE ])

            #Set bluetooth socket to accept incoming connection
            print("Waiting for incoming connection on RFCOMM channel: %d" %port)
            self.clientSocket, self.client_info = self.bluetoothSocket.accept()
            print("Accepted connection from ", self.client_info)
        except Exception as e:
            print("error setting up bluetooth:" + str(e))

    def endConnection(self):
        try:
            if self.clientSocket:
                self.clientSocket.close()
                print("Bluetooth Client connection ended")
            if self.bluetoothSocket:
                self.bluetoothSocket.close()
                print("Bluetooth connection closed")
        except Exception as e:
            print("Error closing connection" + str(e))

    def clientRead(self):
        try:
            data = self.clientSocket.recv(4096)
            return data
        except Exception as e:
            print("Connection error" + str(e))

    def clientWrite(self,Msg):
        try:
            self.clientSocket.send(str(Msg))
        except Exception as e:
            print("Write error" + str(e))

            
if __name__ == "__main__":
    bluetooth_connection = Bluetooth_Connection()
    bluetooth_connection.setupConnection()

    while True:
        clientMsg = bluetooth_connection.clientRead()
        recieveMsg = clientMsg[0:1]
        

        if recieveMsg == "A":
            bluetooth_connection.clientWrite("BT to RPi:" + str(clientMsg[1:]))
            print(recieveMsg)
        elif recieveMsg == "B":
            Msg = raw_input("Enter your message")
            bluetooth_connection.clientWrite("From RPi to BT: " + str(Msg))
            print(Msg)
        elif recieveMsg == "C":
            bluetooth_connection.clientWrite("Connection ended")
            break
        else:
            bluetooth_connection.clientWrite("Rpi: Message received")
    bluetooth_connection.endConnection()
            
