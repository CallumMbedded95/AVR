import serial
import time

ser = serial.Serial('/dev/ttyUSB1', 19200, timeout=2)
ser.flushInput()
ser.flushOutput()
while True:
	ser.write('h')
	print('Writing')
	time.sleep(3)
	data_raw = ser.read()
	print(data_raw)
	print(' '.join(format(ord(x), 'b') for x in data_raw))
	time.sleep(3)