import matplotlib.pyplot as plt
import csv
import numpy as np
from scipy.optimize import curve_fit
import time
import os

x = []
y = []

#--------------------SETTINGS------------------#
number_of_lines = 3
measurement = 1
path = 'CaptureBatCharge_%s.txt' % measurement
plotname = 'PlotBatChargeCurve_%s.pdf' % measurement
begin_sample = 27
end_sample = 2827 %2840
#----------------------------------------------#

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


#   Rescale
for i in range(number_of_samples):
    y[1][i] = y[1][i] / 100
    y[2][i] = y[2][i] / 10

y[0] = [i/60000 for i in y[0]]


sample_print = 200
number_of_samples = int(number_of_samples/sample_print)

#   Test
x = np.linspace(0, number_of_samples, number_of_samples)
a = []
b = []
c = []
for i in range(number_of_samples*sample_print):
    if(i%sample_print == 0):
        print(i)
        a.append(y[1][i])
        b.append(y[2][i])
        c.append(y[0][i])


"""

z = np.polyfit(y[0], y[1], 15)
z2 = np.polyfit(y[0], np.log(y[2]), 1)
p = np.poly1d(z)
p2 = np.poly1d(z2)


a = []
b = []

for i in range(number_of_samples):
    a.append(p(y[0][i]))
    b.append(p2(y[0][i]))

"""

def linesInOnePlot():
    fig = plt.figure(figsize=(15, 10))
    plt.title('Plot %s' % measurement)
    for i in range(1, number_of_lines):
        plt.plot(x, y[i])
    plt.ylabel('Degrees [°]')
    plt.xlabel('Samples [25 Hz]')
    plt.tight_layout()
    fig.savefig(plotname, dpi=100)




fig, ax1 = plt.subplots(figsize=(9, 3), constrained_layout=True)
fig.suptitle('Duration of one charge cycle')
color = 'black'
ax1.set_xlabel('Time [minutes]')
ax1.set_ylabel('Voltage [V]', color=color)
#ax1.plot(y[0], y[1], color=color, label='Battery voltage')
ax1.plot(c, a, color=color, label='Battery voltage')
#ax1.plot(y[0], a, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

color = 'tab:grey'
ax2.set_ylabel('Current [mA]', color=color)  # we already handled the x-label with ax1
#ax2.plot(y[0], y[2], color=color, label='Charge current')
ax2.plot(c, b, color=color, label='Charge current')
#ax2.plot(y[0], c, color=color)
ax2.tick_params(axis='y', labelcolor=color)

ax1.grid(True)

#fig.tight_layout()  # otherwise the right y-label is slightly clipped
fig.legend(bbox_to_anchor=(0.93, 0.78))
fig.savefig(plotname, dpi=100)