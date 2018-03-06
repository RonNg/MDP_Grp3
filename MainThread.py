import sys
import time
import threading
import serial
from Bluetooth_Connection import *
from SerialConnection import *
from WifiConnection import *

class RPIInit(threading.Thread):
	def __init__(self):
		threading.Thread.__init__(self)

		self.SerialConnection = SerialConnection()
		self.WifiConnection = WifiConnection()
		self.BluetoothConnection = Bluetooth_Connection()

		self.WifiConnection.connect()
		self.wifiIsConnected = True
		self.BluetoothConnection.setupConnection()
		self.bluetoothIsConnected = True
		self.SerialConnection.connect()
		self.serialIsConnected = True


	def readFromWifiConnection(self):
		while True:
			try:
				msg = self.WifiConnection.read()
				if msg == '':
					print('Wifi Connection is Lost')
					self.wifiIsConnected = False
					break

				command = msg[0:1]
				print(command)
				print('From PC: ' + str(msg))
				if command == 'A':
					self.SerialConnection.write(str(msg[1:]))
				elif command == 'T':
					self.BluetoothConnection.clientWrite(str(msg[1:]))
				elif command == ';':
					self.WifiConnection.write('RPi3: connection closed')
					self.wifiIsConnected = False
					break
				else:
					self.WifiConnection.write('Rpi3: Message received')
			except Exception as e:
				print('readFromWifiConnection Error' + str(e))
	def readFromBluetoothConnection(self):
		while True:
			try:
				msg = self.BluetoothConnection.clientRead()
				if msg =='':
					print('bluetooth connection lost')
					self.bluetoothIsConnected = False
					break

				command = msg[0:1]
				print(command)
				print('From Bluetooth:' + str(msg))
				if command == 'P':
					self.WifiConnection.write(str(msg[1:]))
				elif command == 'A':
					self.SerialConnection.write(str(msg[1:]))
				elif msg == ';':
					self.BluetoothConnection.clientWrite('RPi3: Connection closed')
					self.bluetoothIsConnected = False
					break
				else:
					self.BluetoothConnection.clientWrite('RPi3: Msg received')
			except Exception as e:
				print('readFromBluetoothConnection Error' + str(e))

	def readFromSerialConnection(self):
		while True:
			try:
				msg = self.SerialConnection.read()
				if msg =='':
					print('serial conncetion is lost')
					self.serialIsConnected = False
					break
				command = msg[0:1]
				print(command)
				print('From Arduino:' + str(msg))
				if command =='P':
					self.WifiConnection.write(str(msg[1:]))
				elif command =='T':
					self.BluetoothConnection.write('RPi3: Instruction ' + str(msg[1:]) + ' Sent to Tablet')
				elif msg == ';':
					self.SerialConnection.Write('RPi3: Connection Closed')
					self.serialIsConnected = False
					break
				else:
					print('works')
			except Exception as e:
				print('readFromSerialConnection Error' + str(e))

	def startThreads(self):
		bluetoothThread = threading.Thread(target = self.readFromBluetoothConnection, name ='Bluetooth Thread')
		wifiThread= threading.Thread(target = self.readFromBluetoothConnection, name="Bluetooth Thread")
		serialThread = threading.Thread(target = self.readFromSerialConnection, name="Serial Thread")

		bluetoothThread.daemon = True
		wifiThread.daemon = True
		serialThread.daemon = True
		print('Susccessfully initialized threads')

		bluetoothThread.start()
		wifiThread.start()
		serialThread.start()
		print("Worker Threads started")

	def cleanup(self):
		try:
			self.WifiConnection.close()
			self.BluetoothConnection.close()
			self.SerialConnection.close()
		except Exception as e:
			print("Cleanup Error:" + str(e))

if __name__ == "__main__":
	print threading.currentThread().getName(), 'Starting'

	RPIInit = RPIInit()
	RPIInit.startThreads()
	RPIInit.readFromWifiConnection()
	print("Awaiting all connections to close")
	while True:
		time.sleep(0.5)
		if (not RPIInit.wifiIsConnected) and (not RPIInit.bluetoothIsConnected):
			RPIInit.cleanup()
			break
					
