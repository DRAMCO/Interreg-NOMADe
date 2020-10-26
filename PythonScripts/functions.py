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
#         File: functions.py
#      Created: 2020-10-06
#       Author: Jarne Van Mulders
#      Version: V1.0
#
#  Description:
#      Plot the data from multiple IMU
#
#  Commissioned by Interreg NOMADe Project

import matplotlib.pyplot as plt
import numpy as np
import csv
import math as m


def load_measurement_data(number_of_lines, y, path):
    for i in range(number_of_lines):
        y.append([])

    with open(path, 'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        for row in plots:
            for i in range(number_of_lines):
                y[i].append(float(row[i]))


def get_data_lines(y):
    return len(y[0])


def get_no_connected_sensors(y, total_number_of_data_rows, sensor_slot_data_available):
    x = 0
    for i in range(1, 7):
        for j in range(total_number_of_data_rows):
            if y[0][j] == i:
                sensor_slot_data_available[i - 1] = 1
                break
    return sensor_slot_data_available.count(1)


def plot_quaternions_all_sensors(connected_sensors, sampling_frequency, sample_list, plot_name):

    lab = ["q0", "q1", "q2", "q3"]
    color = ["blue", "green", "cyan", "grey"]

    fig = plt.figure(figsize=(18, 9))
    for j in range(6):
        if connected_sensors[j]:
            samples = len(sample_list[j][0])
            x = np.linspace(0, samples / 1000, samples)
            plt.subplot(2, 3, j + 1)
            plt.title('Sensor slot %s [%s Hz]' % (int(j + 1), int(sampling_frequency)))
            for i in range(4):
                plt.plot(x, sample_list[j][i], label=lab[i], color=color[i])
            plt.ylabel('Value [-]')
            plt.xlabel('Samples [KS]')
            plt.tight_layout()
            plt.legend()
            plt.grid(True)
    fig.savefig(plot_name, dpi=100)


def plot_ypr_all_sensors(ypr_sample_list, sampling_frequency, connected_sensors, plot_name):

    lab = ["Yaw", "Pitch", "Roll"]
    color = ["blue", "green", "cyan"]

    fig = plt.figure(figsize=(18, 9))
    for j in range(6):
        if connected_sensors[j]:
            samples = len(ypr_sample_list[j][0])
            x = np.linspace(0, samples / 1000, samples)
            plt.subplot(2, 3, j + 1)
            plt.title('Sensor slot %s [%s Hz]' % (int(j+1), int(sampling_frequency)))
            for i in range(3):
                plt.plot(x, ypr_sample_list[j][i], label=lab[i], color=color[i])
            plt.ylabel('Degrees [°]')
            plt.xlabel('Samples [KS]')
            plt.tight_layout()
            plt.legend()
            plt.grid(True)
    fig.savefig(plot_name, dpi=100)


def convert_txt_file(sample_list, y, connected_sensors, no_samples, tot_no_datarows):
    no_variables = 4
    #   Create 3 dimensional list
    create_3d_list(sample_list, no_samples, no_variables)

    #   Seperate samples in 3 dimensional list
    for i in range(0, 6):
        if connected_sensors[i]:
            u = 0
            for j in range(tot_no_datarows):
                if y[0][j] == i + 1:
                    for k in range(no_variables):
                        sample_list[i][k][u] = y[k + 3][j]
                    u = u + 1


def create_3d_list(sample_list, no_samples, no_variables):
    for i in range(6):
        sample_list.append([])
        for j in range(no_variables):
            sample_list[i].append([])
            for k in range(no_samples):
                sample_list[i][j].append(0)


def convert_sample_list_ypr(sample_list, new_sample_list, connected_sensors):
    for k in range(6):
        if connected_sensors[k]:
            transp = np.array(sample_list[k])
            sens_samples = np.transpose(transp)
            clms = len(transp[0])
            sens_samples_ypr = np.zeros((clms, 3))
            for i in range(clms):
                convert_quaternion_sample_to_ypr(sens_samples[i], sens_samples_ypr[i])
            new_sample_list.append(np.transpose(sens_samples_ypr).tolist())
        else:
            new_sample_list.append([])


def convert_sample_list_ypr_degrees(ypr_sample_list, connected_sensors):
    for i in range(6):
        if connected_sensors[i]:
            for k in range(3):
                for l in range(len(ypr_sample_list[i][k])):
                    ypr_sample_list[i][k][l] = ypr_sample_list[i][k][l] * 180/m.pi + 180


def convert_quaternion_sample_to_ypr(data, new_data):
    q = np.zeros((4, 1))
    q[0] = data[0] / 16384.0
    q[1] = data[1] / 16384.0
    q[2] = data[2] / 16384.0
    q[3] = data[3] / 16384.0

    gravity = np.zeros((3, 1))
    gravity[0] = 2 * (q[1] * q[3] - q[0] * q[2])                         #x
    gravity[1] = 2 * (q[0] * q[1] + q[2] * q[3])                         #y
    gravity[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]   #z

    #   yaw: (about Z axis)
    new_data[0] = m.atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1)
    #   pitch: (nose up/down, about Y axis)
    new_data[1] = m.atan2(gravity[0], m.sqrt(gravity[1] * gravity[1] + gravity[2] * gravity[2]))
    #   roll: (tilt left/right, about X axis)
    new_data[2] = m.atan2(gravity[1], gravity[2])

    if gravity[2] < 0:
        if new_data[1] > 0:
            new_data[1] = m.pi - new_data[1]
        else:
            new_data[1] = -m.pi - new_data[1]