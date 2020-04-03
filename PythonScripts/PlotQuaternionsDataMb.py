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
#         File: PlotQuaternionsDataMb.py
#      Created: 2020-04-02
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
from scipy.spatial.transform import Rotation as R
import math as m
from scipy.signal import argrelextrema
import time
import os

x = []
y = []

# --------------------SETTINGS------------------#
number_of_lines = 4
measurement = 2
path = 'Capture_Quaternions_%s.txt' % measurement
plot_name = 'Plot_Quaternions_%s.pdf' % measurement
plot_comb_name = 'PlotComb_Quaternions_%s.pdf' % measurement
plot_tot_name = 'PlotTot_Quaternions_%s.pdf' % measurement
begin_sample = 0
end_sample = 300
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



r = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])










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
        plt.plot(x, y[i], label=lab[i], color=color[i])
    plt.ylabel('Degrees [°]')
    plt.xlim(0, end_sample - begin_sample)
    plt.tight_layout()
    plt.legend()
    plt.grid(True)

    plt.subplot(212)
    plt.title('3D rotations reduced to one dimension')
    plt.plot(np.linspace(0, len(a), len(a)), a, color='black')
    plt.ylabel('Intensity')
    plt.xlabel('Samples [25 Hz]')
    plt.xlim(0, end_sample - begin_sample)
    plt.tight_layout()
    plt.legend()
    plt.grid(True)

    fig.savefig(plot_tot_name, dpi=100)


totalPlot()
# linesInOnePlot()
# directionsCombined()