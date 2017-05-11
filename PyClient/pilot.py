#!/usr/bin/python3

import serial
import time
from time import sleep
import sys
import matplotlib.pyplot as plt
import numpy
import re

current_milli_time = lambda: int(round(time.time() * 1000))

def assemble_packet(payload):
    bt_packet = bytearray()
    payload_size = len(payload)
    if payload_size >= 120:
        print("WARNING: Payload size should be less than 120 [TRUNCATED]")
        payload_size = payload_size % 120

    bt_packet.append(BT_P_HEADER)
    bt_packet.append(id)
    bt_packet.append(payload_size)
    crc = BT_P_HEADER ^ id ^ payload_size
    for i in range(0, payload_size):
        bt_packet.append(payload[i])   # payload[i] should be an int [0, 255]
        crc ^= payload[i]
    bt_packet.append(BT_P_TAIL)
    crc ^= BT_P_TAIL
    bt_packet.append(crc)
    return bt_packet

def int2short(i):
    if i > (1<<15) - 1:
        return i - (1<<16)
    else:
        return i

def decode_ack(packet_ba, payload):
    pass
            
serial_port = '/dev/ttyACM0'

ser = serial.Serial(
    port=serial_port,
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS)

if ser.isOpen():
    print("Serial {0} Opened.".format(serial_port))

ser.flushInput()
ser.flushOutput()

while True:
    for length in range(1, 32):
        spacket = bytearray()
        spacket.append(0xE0 | length)
        for i in range(0, length):
            spacket.append(i)
        ser.write(spacket)
        #print("Sending Packet ... {0}".format(str(spacket)))
        input("next")
        while True:
            a = ser.inWaiting()
            if a == 0:
                break
            print("{0:x} ".format(ord(ser.read(1))), end='')
        print('')

