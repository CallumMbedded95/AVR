import serial
import time

ser = serial.Serial('/dev/ttyUSB1', 19200, timeout=2)
ser.flushInput()
ser.flushOutput()
while True:
	ser.write('a')
	print('Writing')
	time.sleep(1)
	data_raw = ser.read()
	print(data_raw)
	print(' '.join(format(ord(x), 'b') for x in data_raw))
	time.sleep(1)