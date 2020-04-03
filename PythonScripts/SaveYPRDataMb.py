#  ____  ____      _    __  __  ____ ___
# |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
# | | | | |_) |  / _ \ | |\/| | |  | | | |
# | |_| |  _ <  / ___ \| |  | | |__| |_| |
# |____/|_| \_\/_/   \_\_|  |_|\____\___/
#                           research group
#                             dramco.be/
#
#  KU Leuven - Technology Campus Gent,
#  Gebroeders De Smetstraat 1,
#  B-9000 Gent, Belgium
#
#         File: SaveYPRDataMb.py
#      Created: 2020-03-19
#       Author: Jarne Van Mulders
#      Version: V1.0
#
#  Description:
#      Save the data from one IMU
#      Data: Yaw - Pitch - Roll
#
#  Commissiond by Interreg NOMADe Project
#


import serial
from time import sleep
import time
import os
import numpy as np
from datetime import datetime

#--------------------SETTINGS------------------#
COM = 'COM5'  # /dev/ttyACM0 (Linux)
BAUD = 1000000
number_of_lines = 3
measurement = 6
path = 'Capture_%s.txt' % measurement
#----------------------------------------------#

#   Open Serial Monitor #
ser = serial.Serial(COM, BAUD)

#   Check Serial Monitor #
print('Waiting for device')
sleep(1)
print(ser.name)

#   If not excist, create file
f = open(path, "w")
f.close()


count = 0

def getValue():
    value = (number_of_lines)
    value = np.zeros(value)
    while (1):
        num = ser.inWaiting()
        if (num > 0):
            a = int.from_bytes(ser.read(1), "big")
            print(a)
            if(a == 2):
                msg = ser.read(9)
                for k in range(number_of_lines):
                    value[k] = int.from_bytes([msg[2 + k * 2], msg[3 + k * 2]], "little")
                print(num)
                print(value)
                #print(datetime.timestamp(datetime.now()))
                break
    return value

while 1:
    f = open(path, "a")
    value = getValue()
    for i in range(number_of_lines):
        f.write(str(value[i]))
        f.write(",")
    f.write("\n")
    f.close()

