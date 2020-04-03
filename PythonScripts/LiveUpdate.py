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
#         File: LiveUpdate.py
#      Created: 2020-03-19
#       Author: Jarne Van Mulders
#      Version: V1.0
#
#  Description:
#      Live Visualisation of the data from one IMU
#       Yaw - Pitch - Roll
#
#  Commissiond by Interreg NOMADe Project
#


import serial
from time import sleep
import time
import os
import matplotlib; matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

#--------------------SETTINGS------------------#
COM = 'COM5'  # /dev/ttyACM0 (Linux)
BAUD = 1000000
number_of_samples_visable = 200
number_of_lines = 3
#----------------------------------------------#

fig = plt.figure(figsize = [20, 10], dpi=75)
#ax1 = fig.add_subplot(2, 2, 1)

lines = [plt.plot([], [])[0] for _ in range(number_of_lines)] #lines to animate

# Open Serial Monitor #
ser = serial.Serial(COM, BAUD)

# Check Serial Monitor #
print('Waiting for device')
sleep(1)
print(ser.name)

y = (number_of_lines, number_of_samples_visable)
y = np.zeros(y)

rotations = (number_of_lines)
rotations = np.zeros(rotations)

val = (number_of_lines)
val = np.zeros(val)

value_old = np.array([180,180,180])

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
                break
    return value

def valueInclRot():
    global value_old, val
    value = getValue()

    for i in range(number_of_lines):
        val [i] = value_old[i] - value[i]

    value_old = np.copy(value)

    for i in range(number_of_lines):
        if(val [i] > 330):
            rotations[i] += 1
        if(val [i] < -330):
            rotations[i] -= 1

        value[i] = value[i] + rotations[i] * 360

    return value


def init():
    #init lines
    for line in lines:
        line.set_data([], [])

    return lines #return everything that must be updated

def animate(i):
    global count
    value = getValue()
    #value = valueInclRot()

    if (count < number_of_samples_visable):
        for k in range(number_of_lines):
            y[k][count] = value[k]
    else:


        for k in range(number_of_lines):
            y[k] = np.roll(y[k], -1)
            y[k][number_of_samples_visable-1] = value[k]




    for j, line in enumerate(lines):
        line.set_data(np.linspace(0, number_of_samples_visable - 1, number_of_samples_visable), [y[j]])

    count += 1
    return lines    #return everything that must be updated

#anim = animation.FuncAnimation(fig, animate, init_func=init, frames=100, interval=20, blit=True)
ani = animation.FuncAnimation(fig, animate, frames = 50, init_func=init, interval=10, blit=True, save_count=number_of_samples_visable)


# Open Plot #
plt.xlim(0, number_of_samples_visable)
plt.ylim(0, 360)
plt.xlabel("Samples")
plt.ylabel("Acc ADC value")
plt.title("Accelerometer live update")
plt.show()
