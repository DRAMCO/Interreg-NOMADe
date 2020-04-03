import numpy as np
import math

roll    = 20/ 180 * math.pi     #85 / 180 * math.pi
pitch   = 85/ 180 * math.pi   #-55 / 180 * math.pi
yaw     = -10/ 180 * math.pi    #-10 #/ 180 * math.pi

roll    = 83/ 180 * math.pi     #85 / 180 * math.pi
pitch   = 21/ 180 * math.pi   #-55 / 180 * math.pi
yaw     = 38/ 180 * math.pi    #-10 #/ 180 * math.pi

roll    = 94/ 180 * math.pi     #85 / 180 * math.pi
pitch   = 46/ 180 * math.pi   #-55 / 180 * math.pi
yaw     = 17/ 180 * math.pi    #-10 #/ 180 * math.pi

alpha = yaw
beta = pitch
gamma = roll


print("roll = ", roll)
print("pitch = ", pitch)
print("yaw = ", yaw)
print("")

#       Determine the rotation matrix       #
rot = np.array([[np.cos(alpha)*np.cos(beta), np.cos(alpha)*np.sin(beta)*np.sin(gamma) - np.sin(alpha)*np.cos(gamma), np.cos(alpha)*np.sin(beta)*np.cos(gamma) + np.sin(alpha)*np.sin(gamma)],
          [np.sin(alpha)*np.cos(beta), np.sin(alpha)*np.sin(beta)*np.sin(gamma) + np.cos(alpha)*np.cos(gamma), np.sin(alpha)*np.sin(beta)*np.cos(gamma) - np.cos(alpha)*np.sin(gamma)],
          [-np.sin(beta), np.cos(beta)*np.sin(gamma), np.cos(beta)*np.cos(gamma)]])


def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


vector1 = np.array([0, 0, 1])
vector2 = np.matmul(vector1, rot)
print(vector1)
print(vector2)
print(angle_between(vector1, vector2)*180/math.pi)

