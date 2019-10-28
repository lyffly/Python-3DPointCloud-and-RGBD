from transforms3d.quaternions import quat2mat,mat2quat
from transforms3d.euler import mat2euler,euler2mat,euler2quat,quat2mat
import numpy as np
np.set_printoptions(precision=3, suppress=True)  # neat printing

a = [0,0,0,1]
a1 = quat2mat(a)
print(a1)

a = [0,0,1,0]
a1 = quat2mat(a)
print(a1)

a = [0,1,0,0]
a1 = quat2mat(a)
print(a1)

a = [1,0,0,0]
a1 = quat2mat(a)
print(a1)

x_angle = -0.2
y_angle = -np.pi/2
z_angle = -0.2
R = euler2mat(x_angle,y_angle,z_angle,axes='sxyz')
print(R)

