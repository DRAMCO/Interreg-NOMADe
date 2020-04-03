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
#         File: PlotDataMainboard.py
#      Created: 2020-03-19
#       Author: Jarne Van Mulders
#      Version: V1.0
#
#  Description:
#      Plot the data from one IMU
#
#  Commissiond by Interreg NOMADe Project
#

import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.signal import find_peaks
import math as m
from scipy.signal import argrelextrema
import time
import os

x = []
y = []

# --------------------SETTINGS------------------#
number_of_lines = 3
measurement = 6
path = 'Capture_%s.txt' % measurement
plot_name = 'Plot_%s.pdf' % measurement
plot_comb_name = 'PlotComb_%s.pdf' % measurement
plot_tot_name = 'PlotTot_%s.pdf' % measurement
begin_sample = 100
end_sample = 1000
threshold = 130
# ----------------------------------------------#

for i in range(number_of_lines):
    y.append([])

with open(path, 'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        for i in range(number_of_lines):
            y[i].append(float(row[i]))

for i in range(number_of_lines):
    y[i] = y[i][begin_sample:end_sample]

number_of_samples = len(y[0])
print(number_of_samples)
x = np.linspace(0, number_of_samples, number_of_samples)

"""
print(y[0]) # Yaw
print(y[1]) # Pitch
print(y[2]) # Roll
"""
print("y[0] = ", max(y[0]) - min(y[0]))
print("y[1] = ", max(y[1]) - min(y[1]))
print("y[2] = ", max(y[2]) - min(y[2]))


###########################################################
###                 Extra berekeningen                  ###
###########################################################
def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


a = []

#           ***     Algoritme kijken oefening goed/slecht       ***

max_peaks_indices, _ = find_peaks(y[2], height=0)

min_peaks = []
min_peaks_indices = [max_peaks_indices[0]]

for i in range(len(max_peaks_indices) - 1):
    # min_peaks.append(min(y[2][peaks[i]:peaks[i + 1]]))
    new_block = y[2][max_peaks_indices[i]:max_peaks_indices[i + 1]]
    min_value = min(new_block)
    min_peaks_indices.append(max_peaks_indices[i] + new_block.index(min_value))
    # min_peaks_ind.append(min_peaks_ind[i] + y[2][peaks[i]:peaks[i + 1]].index(min(y[2][peaks[i]:peaks[i + 1]])))
min_peaks_indices = np.array(min_peaks_indices[1:])

number_of_peaks = min([len(max_peaks_indices), len(min_peaks_indices)])
max_peaks_indices = max_peaks_indices[0:number_of_peaks]
min_peaks_indices = min_peaks_indices[0:number_of_peaks]

print(max_peaks_indices)
print(min_peaks_indices)
print(number_of_peaks)

for i in range(number_of_peaks):
    #   Bepaal de hoekverdraaiiing tussen een maximum en een minimum
    yaw = (y[0][max_peaks_indices[i]] - y[0][min_peaks_indices[i]])
    pitch = (y[1][max_peaks_indices[i]] - y[1][min_peaks_indices[i]])
    roll = (y[2][max_peaks_indices[i]] - y[2][min_peaks_indices[i]])

    print(yaw, pitch, roll)

    alpha = yaw / 180 * m.pi
    beta = pitch / 180 * m.pi
    gamma = roll / 180 * m.pi
    """
    alpha = yaw / 180 * m.pi
    beta = pitch / 180 * m.pi
    gamma = roll / 180 * m.pi
    """

    #       Determine the rotation matrix       #
    rot = np.array([[np.cos(alpha) * np.cos(beta),
                     np.cos(alpha) * np.sin(beta) * np.sin(gamma) - np.sin(alpha) * np.cos(gamma),
                     np.cos(alpha) * np.sin(beta) * np.cos(gamma) + np.sin(alpha) * np.sin(gamma)],
                    [np.sin(alpha) * np.cos(beta),
                     np.sin(alpha) * np.sin(beta) * np.sin(gamma) + np.cos(alpha) * np.cos(gamma),
                     np.sin(alpha) * np.sin(beta) * np.cos(gamma) - np.cos(alpha) * np.sin(gamma)],
                    [-np.sin(beta), np.cos(beta) * np.sin(gamma), np.cos(beta) * np.cos(gamma)]])

    vector1 = np.array([0, 0, 1])
    vector2 = np.matmul(vector1, rot)
    print(vector1)
    print(vector2)
    a.append(angle_between(vector1, vector2) * 180 / m.pi)

print(a)

b = np.ones(number_of_peaks)*threshold

plt.plot(max_peaks_indices, a)
plt.plot(max_peaks_indices, b)
plt.show()




"""
----  3e Versie  ---- 

for i in range(len(y[0])):
    gamma   = y[0][i] / 180 * m.pi
    beta    = y[1][i] / 180 * m.pi
    alpha   = y[2][i] / 180 * m.pi

    #   Bepaal de rotatiematrix tussen een maximum en een minimum
    rot = np.array([[np.cos(alpha) * np.cos(beta),
                     np.cos(alpha) * np.sin(beta) * np.sin(gamma) - np.sin(alpha) * np.cos(gamma),
                     np.cos(alpha) * np.sin(beta) * np.cos(gamma) + np.sin(alpha) * np.sin(gamma)],
                    [np.sin(alpha) * np.cos(beta),
                     np.sin(alpha) * np.sin(beta) * np.sin(gamma) + np.cos(alpha) * np.cos(gamma),
                     np.sin(alpha) * np.sin(beta) * np.cos(gamma) - np.cos(alpha) * np.sin(gamma)],
                    [-np.sin(beta), np.cos(beta) * np.sin(gamma), np.cos(beta) * np.cos(gamma)]])

    vector1 = np.array([1, 1, 1])
    vector2 = np.matmul(vector1, rot)
    a.append(angle_between(vector1, vector2) * 180 / m.pi)

print(a)



plt.plot(np.linspace(0, len(a), len(a)), a)

plt.show()

"""

"""
----  2e Versie  ---- 

def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))

a = []

#           ***     Algoritme kijken oefening goed/slecht       ***

peaks, _ = find_peaks(y[2], height=0)
print(peaks)

min_peaks = []

for i in range(len(peaks)-1):
    min_peaks.append(y[2].index(min(y[2][peaks[i]:peaks[i+1]])))



for i in range(len(min_peaks)):
    
    #   Bepaal de hoekverdraaiiing tussen een maximum en een minimum
    gamma = (y[0][peaks[i]] - y[0][min_peaks[i]]) / 180 * m.pi
    beta = (y[1][peaks[i]] - y[1][min_peaks[i]]) / 180 * m.pi
    alpha = (y[2][peaks[i]] - y[2][min_peaks[i]]) / 180 * m.pi

    print(alpha, beta, gamma)

    #   Bepaal de rotatiematrix tussen een maximum en een minimum
    rot = np.array([[np.cos(alpha) * np.cos(beta),
                     np.cos(alpha) * np.sin(beta) * np.sin(gamma) - np.sin(alpha) * np.cos(gamma),
                     np.cos(alpha) * np.sin(beta) * np.cos(gamma) + np.sin(alpha) * np.sin(gamma)],
                    [np.sin(alpha) * np.cos(beta),
                     np.sin(alpha) * np.sin(beta) * np.sin(gamma) + np.cos(alpha) * np.cos(gamma),
                     np.sin(alpha) * np.sin(beta) * np.cos(gamma) - np.cos(alpha) * np.sin(gamma)],
                    [-np.sin(beta), np.cos(beta) * np.sin(gamma), np.cos(beta) * np.cos(gamma)]])

    vector1 = np.array([1, 1, 1])
    vector2 = np.matmul(vector1, rot)
    a.append(angle_between(vector1, vector2) * 180 / m.pi)

print(a)

b = [75, 75, 75, 75]

plt.plot(np.linspace(0, len(a), len(a)), a)
plt.plot(np.linspace(0, len(a), len(a)), b)
plt.show()

"""

"""
----  1e Versie  ---- 
#           ***     Combinatie van 3 richtingen     ***
a = []
for i in range(number_of_samples):
    a.append(np.sqrt((np.square(y[0][i]) + np.square(y[1][i]) + np.square(y[2][i]))))

#           ***     Algoritme kijken oefening goed/slecht       ***

peaks, _ = find_peaks(a, height=0)
print(peaks)

print(len(peaks))
print(peaks[2])

min_peaks = []
min_peak_values = []
max_peak_values = []

for i in range(len(peaks)-1):
    min_peaks.append(a.index(min(a[peaks[i]:peaks[i+1]])))
    min_peak_values.append(min(a[peaks[i]:peaks[i+1]]))
    max_peak_values.append(a[peaks[i]])
print(min_peaks)
print(min_peak_values)
print(max_peak_values)

range_movement_in_degrees = []
for i in range(len(peaks)-1):
    range_movement_in_degrees.append(abs(a[peaks[i]] - min_peak_values[i]))

x1 = np.linspace(0, len(range_movement_in_degrees), len(range_movement_in_degrees))

##PLOT FIGURE##
plt.plot(x1, range_movement_in_degrees)
plt.show()
"""

###########################################################


lab = ["Yaw", "Pitch", "Roll"]
color = ["darkgrey", "black", "grey"]


def linesInOnePlot():
    fig = plt.figure(figsize=(9, 3))
    plt.title('Plot %s' % measurement)
    for i in range(number_of_lines):
        plt.plot(x, y[i], label=lab[i], color=color[i])
    plt.ylabel('Degrees [°]')
    plt.xlabel('Samples [25 Hz]')
    plt.xlim(0, end_sample - begin_sample)
    plt.tight_layout()
    plt.legend()
    plt.grid(True)
    fig.savefig(plot_name, dpi=100)


def directionsCombined():
    a = []
    for i in range(number_of_samples):
        a.append(np.sqrt((np.square(y[0][i]) + np.square(y[1][i]) + np.square(y[2][i]))))

    fig = plt.figure(figsize=(9, 3))
    plt.title('Plot %s' % measurement)
    plt.plot(x, a, color='black')
    plt.ylabel('Intensity')
    plt.xlabel('Samples [25 Hz]')
    plt.xlim(0, end_sample - begin_sample)
    plt.tight_layout()
    plt.legend()
    plt.grid(True)
    fig.savefig(plot_comb_name, dpi=100)


def totalPlot():
    fig = plt.figure(figsize=(9, 6))
    plt.subplot(211)
    plt.title('Rotations: Yaw Pitch Roll')
    for i in range(number_of_lines):
        plt.plot(x/25, y[i], label=lab[i], color=color[i])
    plt.ylabel('Degrees [°]')
    plt.xlim(0, (end_sample - begin_sample)/25)
    plt.tight_layout()
    plt.legend()
    plt.grid(True)

    plt.subplot(212)
    plt.title('3D rotations reduced to one dimension')
    plt.plot(max_peaks_indices/25, a, color='black')
    plt.plot(max_peaks_indices/25, b, color='grey')
    plt.ylabel('Intensity')
    #plt.xlabel('Samples [25 Hz]')
    plt.xlabel('Time [s]')
    plt.xlim(0, (end_sample - begin_sample)/25)
    plt.ylim(100, max(a)*1.05)
    plt.tight_layout()
    plt.legend()
    plt.grid(True)

    fig.savefig(plot_tot_name, dpi=100)

totalPlot()
# linesInOnePlot()
# directionsCombined()
