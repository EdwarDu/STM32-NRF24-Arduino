#!/usr/bin/python3

import serial
import time
import sys
import matplotlib.pyplot as plt
import numpy
import re

serial_port = '/dev/ttyUSB0'

ser = serial.Serial(
	port=serial_port,
	baudrate=115200,
	parity=serial.PARITY_NONE,
	stopbits=serial.STOPBITS_ONE,
	bytesize=serial.EIGHTBITS )

if ser.isOpen():
	print("Serial {0} Opened.".format(serial_port))

rec_index = []
yaw = []
pitch = []
roll = []

#plt.ion()
hyaw, hpitch, hroll = plt.plot(rec_index, yaw, 'r-', 
								rec_index, pitch, 'g-', 
								rec_index, roll, 'b-',)

plt.ylim(-200, 200)

print("PyPlot Show!")
ax = plt.gca()
record_counter = 0

while(True):
	line = ser.readline().decode('utf-8')[:-2]
	print(line)
	if (line.startswith("ready")):
		break

while(True):
	ser.write(bytearray('c', 'ascii'))
	line = ser.readline().decode('utf-8')[:-2]

	try:
		print(line);
		y, p, r= [p for p in re.split(" |\t", line) if p != ""]
		yaw.append(float(y))
		pitch.append(float(p))
		roll.append(float(r))
		rec_index.append(record_counter)
		hyaw.set_xdata(rec_index)
		hyaw.set_ydata(yaw)
		hpitch.set_xdata(rec_index)
		hpitch.set_ydata(pitch)
		hroll.set_xdata(rec_index)
		hroll.set_ydata(roll)
		plt.xlim(rec_index[0], rec_index[-1])
		ax.autoscale_view()
		plt.draw()
		plt.pause(0.01)
		record_counter += 1
	except (RuntimeError, TypeError, NameError, ValueError) as err:
		print("Oops, error! {0}".format(str(err)))
		if (len(yaw) != record_counter):
			yaw.pop(-1)
		if (len(pitch) != record_counter):
			pitch.pop(-1)
		if(len(roll) != record_counter):
			roll.pop(-1)
	else:
		if len(yaw) > 300:
			yaw.pop(0)
			pitch.pop(0)
			roll.pop(0)
			rec_index.pop(0)
