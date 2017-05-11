#!/usr/bin/python3

import serial
import time
from time import sleep
import sys
import matplotlib.pyplot as plt
import numpy
import re

current_milli_time = lambda: int(round(time.time() * 1000))

BT_P_HEADER     = 0xBB
BT_P_TAIL       = 0xEE

BT_ACK_HEADER   = 0xBA
BT_ACK_TAIL     = 0xAE

DATA_ACQ_CMD          =     0x81
DF1_GYRO_XYZ          =     0x01
DF1_ACCL_XYZ          =     0x02
DF1_COMP_XYZ          =     0x04
DF1_MPU_YPR           =     0x08
DF1_DOF_ALL           =     0x0F

DF1_BATT_VOLTAGE      =     0x10
DF1_MOTOR_1234        =     0x20
DF1_DIST              =     0x40

DF2_MPU_STATUS        =      0x01
DF2_NRF24_STATUS      =      0x02
DF2_SYSTICK           =      0x04

MPU_INIT_GOOD         =      0x80
MPU_DMP_GOOD          =      0x40

MPU_ST_GYRO_GOOD      =      0x02
MPU_ST_ACCL_GOOD      =      0x01
MPU_ST_COMP_GOOD      =      0x04

NRF24_CHECK_GOOD      =      0x80

MOTOR_CTRL_CMD          = 0x82

are_we_ready = False

time_ax = []
yaw = []
pitch = []
roll = []

#plt.ion()
hyaw, hpitch, hroll = plt.plot(time_ax, yaw, 'r-', 
                                time_ax, pitch, 'g-', 
                                time_ax, roll, 'b-',)
ax = plt.gca()

def assemble_packet(id, payload):
    bt_packet = bytearray()
    id = id % 256
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
    if packet_ba[0] != BT_ACK_HEADER or packet_ba[-1] != BT_ACK_TAIL:
        return (-1, )
    id = packet_ba[1]
    size = packet_ba[2]
    ack_payload = packet_ba[3:-1]

    index = 0
    if payload[0] == DATA_ACQ_CMD:
        if payload[1] & DF1_GYRO_XYZ:
            x = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            y = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            z = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            print("gyro: {0} {1} {2}".format(x, y, z))
        if payload[1] & DF1_ACCL_XYZ:
            x = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            y = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            z = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            print("accel: {0} {1} {2}".format(x, y, z))
        if payload[1] & DF1_COMP_XYZ:
            x = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            y = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            z = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            print("compass: {0} {1} {2}".format(x, y, z))
        if payload[1] & DF1_MPU_YPR:
            x = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            y = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            z = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            print("ypr: {0} {1} {2}".format(x/100.0, y/100.0, z/100.0))
            global yaw, pitch, roll
            yaw.append(x/100.0)
            pitch.append(y/100.0)
            roll.append(z/100.0)
            hyaw.set_xdata(time_ax)
            hyaw.set_ydata(yaw)
            hpitch.set_xdata(time_ax)
            hpitch.set_ydata(pitch)
            hroll.set_xdata(time_ax)
            hroll.set_ydata(roll)
        if payload[1] & DF1_DOF_ALL:
            ts = (ack_payload[index] << 24) + (ack_payload[index+1] << 16) + (ack_payload[index+2] << 8) + ack_payload[index+3]
            index += 4
            print("timestamp: {0} ms".format(ts))
            time_ax.append(ts)
            global ax
            plt.xlim(time_ax[0], time_ax[-1])
            ax.autoscale_view()
            plt.draw()
            plt.pause(0.001)
        if payload[1] & DF1_BATT_VOLTAGE:
            adc_value = int2short((ack_payload[index] << 8) + ack_payload[index+1])
            index += 2
            print("battery: {0:.2f} v".format(adc_value * 5 * 3.3 / 4096))
        if payload[1] & DF1_MOTOR_1234:
            motor1 = ack_payload[index]
            motor2 = ack_payload[index+1]
            motor3 = ack_payload[index+2]
            motor4 = ack_payload[index+3]
            index += 4
            print("motors: {0} {1} {2} {3}".format(motor1, motor2, motor3, motor4))
        if payload[1] & DF1_DIST:
            if ack_payload[index] == 0xFF and ack_payload[index+1] == 0xFF:
                print("dist: OOM (out of measurement range(user)")
            elif ack_payload[index] == 0xFE and ack_payload[index+1] == 0xFE:
                print("dist: TIMEOUT")
            else:
                dist = (ack_payload[index] << 8) + ack_payload[index+1]
                print("dist: {0} mm".format(dist))
            index += 2

        if len(payload) > 2:
            bPass = True
            if payload[2] & DF2_MPU_STATUS:
                if ack_payload[index] & MPU_INIT_GOOD:
                    print("MPU: Init Good")
                else:
                    print("MPU: Init Failed")
                    bPass = False
                print("MPU ST: ", end = '')
                if ack_payload[index] & MPU_ST_GYRO_GOOD:
                    print("Gyro ", end = '')
                else:
                    bPass = False
                if ack_payload[index] & MPU_ST_ACCL_GOOD:
                    print("Accel ", end = '')
                else:
                    bPass = False
                if ack_payload[index] & MPU_ST_COMP_GOOD:
                    print("Compass ", end = '')
                else:
                    bPass = False
                print("PASS")
                if ack_payload[index] & MPU_DMP_GOOD:
                    print("MPU: DMP Good")
                else:
                    print("MPU: DMP Failed")
                    bPass = False
                index += 1
                if bPass:
                    global are_we_ready
                    are_we_ready = True
            if payload[2] & DF2_NRF24_STATUS:
                if ack_payload[index] & NRF24_CHECK_GOOD:
                    print("NRF24: Check Good")
                else:
                    print("NRF24: Check Failed")
                    # I don't care for this for now
                index += 1
            if payload[2] & DF2_SYSTICK:
                ts = (ack_payload[index] << 24) + (ack_payload[index+1] << 16) + (ack_payload[index+2] << 8) + ack_payload[index+3]
                #ts = (ack_payload[index] << 8) + ack_payload[index+1]
                print("On Board TS: {0} ms".format(ts/1000))
                index += 4
            
serial_port = '/dev/ttyUSB0'

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

payload_test_init = [DATA_ACQ_CMD, 0x00, DF2_MPU_STATUS | DF2_NRF24_STATUS | DF2_SYSTICK]
payload_test = [DATA_ACQ_CMD, DF1_DIST | DF1_BATT_VOLTAGE]
payload_minThro = [MOTOR_CTRL_CMD, 0x00, 0x00, 0x00, 0x00]
payload_maxThro = [MOTOR_CTRL_CMD, 0xFF, 0xFF, 0xFF, 0xFF]

count = 1000

id = 0
t1 = 0
t2 = 0
tcur = 0
TIMEOUT_MS = 1000
ack_msg = []
ack_msg_payload_size = 0

plt.ylim(-200, 200)

sleep(2) # sleep 2s for the arduino to restart

payload = payload_test_init

while count:
    count -= 1

#    if are_we_ready:
#        payload = payload_test
#    choice = input("Press h to send max throttle / l to send min throttle / s to send 0x20: ")
#    if (choice == 'h'):
#        print("Max throttle it is")
#        payload = payload_maxThro
#    elif choice == 'l':
#        print("Min throttle it is")
#        payload = payload_minThro
#    elif choice == 's':
#        print("0x20 it is")
#        payload = payload_testThro
#    else:
#        print("Send Test")

    payload = payload_test

    #payload = [MOTOR_CTRL_CMD, id, id, id, id]

    bt_p = assemble_packet(id, payload)
    ser.write(bt_p)
    
    t1 = current_milli_time()
    while current_milli_time() - t1 < TIMEOUT_MS:
        if ser.inWaiting() > 0:
            b = ord(ser.read(1))
            if b == BT_ACK_HEADER:
                ack_msg.append(b)
                break
    else:
        print("ACK TIMED OUT (1)")
        
    while current_milli_time() - t1 < TIMEOUT_MS:
        if ser.inWaiting() > 0:
            b = ord(ser.read(1))
            ack_msg.append(b)
            if 3 == len(ack_msg):
                ack_msg_payload_size = ack_msg[2]

            if len(ack_msg) == ack_msg_payload_size + 4: 
                if b == BT_ACK_TAIL:
                    t2 = current_milli_time()
                    print("+{0}ms ACK RECV".format(t2-t1))
                    decode_ack(ack_msg, payload)
                    break
                else:
                    print("+{0}ms ACK RECV({1} {2}) but corrupted".format(t2-t1, ack_msg[1],
                        ack_msg[2]))

    else:
        print("ACK TIMED OUT (2)")
    id = (id+1) % 256
    ack_msg.clear()
    sleep(1)
