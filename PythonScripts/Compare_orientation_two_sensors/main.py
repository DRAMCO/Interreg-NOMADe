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
#         File: main.py
#      Created: 2020-10-06
#       Author: Jarne Van Mulders
#      Version: V1.0
#
#  Description:
#      Plot the data from multiple IMU
#
#  Commissioned by Interreg NOMADe Project

from functions import *

# --------------------SETTINGS------------------#
measurement_number = 157
sampling_frequency = 100
# ----------------------------------------------#

# ------------ANGLE BETWEEN SENSORS-------------#
first_sensor = 2
second_sensor = 4
# ----------------------------------------------#


no_lines = 7

path = 'MES_%s.txt' % measurement_number
plot_name = 'GRAF_%s.pdf' % measurement_number
plot_name_ypr = 'GRAF_%s_ypr.pdf' % measurement_number
plot_name_diff = 'GRAF_%s_diff.pdf' % measurement_number

y = []

#   Load measurement data from TXT file
load_measurement_data(no_lines, y, path)

#   Update total number of datarows in the TXT file
tno_data_rows = get_data_lines(y)
print(tno_data_rows)

#   Determine how many sensor modules where connected
connected_sensors = [0] * 6
no_connected_sensors = get_no_connected_sensors(y, tno_data_rows, connected_sensors)
print("Number of sensors: %s" % no_connected_sensors)
print(connected_sensors)

#   Number of samples for each sensor
no_samples = int((max(y[1])+1)*10)



#   Create 3 dimensional list with the samples from the individual sensors
sample_list = []
convert_txt_file(sample_list, y, connected_sensors, no_samples, tno_data_rows)

#   You can find the samples from sensor 1 in   <<< sample_list [0] >>>
#   You can find the samples from sensor 2 in   <<< sample_list [1] >>>
#   ...
#   Quaternion number 0 from sensor 2 sample 132 can be accessed with   <<< sample_list [1][0][132]
#   Quaternion number 1 from sensor 2 sample 132 can be accessed with   <<< sample_list [1][1][132]
#   ...

difference_angle_list = []

#   A few plot and convert examples

#   Plot quaternions
plot_quaternions_all_sensors(connected_sensors, sampling_frequency, sample_list, plot_name)

#   Convert quaternions to YPR (in rad)
ypr_sample_list = []
convert_sample_list_ypr(sample_list, ypr_sample_list, connected_sensors)


#   Convert YPR (in rad) to YPR (in degrees)
convert_sample_list_ypr_degrees(ypr_sample_list, connected_sensors)

#   Plot YPR
plot_ypr_all_sensors(ypr_sample_list, sampling_frequency, connected_sensors, plot_name_ypr)


# Plot difference between 2 sensors YPR (in degrees)
plot_relative_angle_sensors(connected_sensors, sampling_frequency, sample_list, plot_name_diff, first_sensor, second_sensor)

