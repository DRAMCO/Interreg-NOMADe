from scipy.spatial.transform import Rotation as R
import numpy as np
import math as m


def cal_euler(q):
    euler = [m.atan2(2 * q[1] * q[2] - 2 * q[0] * q[3], 2 * q[0] * q[0] + 2 * q[1] * q[1] - 1),
             -m.asin(2 * q[1] * q[3] + 2 * q[0] * q[2]),
             m.atan2(2 * q[2] * q[3] - 2 * q[0] * q[1], 2 * q[0] * q[0] + 2 * q[3] * q[3] - 1)]
    return euler


r1 = R.from_quat([14969.0 / 16384.0, 884.0 / 16384.0, 64498.0 / 16384.0, 6519.0 / 16384.0])
e1 = r1.as_euler('zyx', degrees=True)
print(e1)
a = {14969.0 / 16384.0, 884.0 / 16384.0, 64498.0 / 16384.0, 6519.0 / 16384.0}
print(cal_euler(a))

r2 = R.from_quat([15427.0, 23.0, 64024.0, 60230.0])
e2 = r2.as_euler('zyx', degrees=True)

print(e1)
print(e2)

"""

e = np.subtract(e1, e2)
print(e)

rn = R.from_euler('zyx', e, degrees=True)


v = np.array([1, 0, 0])
ev = np.subtract(v, e)
print(ev)

eva = rn.apply(v)

print(eva)


v1 = r1.apply(v)
v2 = r2.apply(v)

print(v1)
print(v2)

print(np.subtract(v1, v2))

def unit_vector(vector):
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


print(angle_between(v, np.array([0, 1, 1])) * 180 / m.pi)
"""
