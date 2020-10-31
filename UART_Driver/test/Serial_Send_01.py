import serial
import time

ser = serial.Serial('/dev/ttyUSB1', 9600, timeout=2)
ser.flushInput()
ser.flushOutput()
while True:
	ser.write('h')
	time.sleep(1)
# while True:
# 	data_raw = ser.read()
# 	#data_raw.encode('ascii')
# 	print(data_raw)
# 	print(' '.join(format(ord(x), 'b') for x in data_raw))
	#print(byte_list.append(binary_representation))