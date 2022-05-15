import numpy as np
import math

T_cam_lidar = np.matrix([[ 0.01666841, -0.99983836, -0.00673865, -0.19164723],
        [-0.16942405,  0.00381777, -0.98553585, -0.27016748],
        [ 0.98540228,  0.01756901, -0.16933303, -0.39299324],
        [ 0.,         0.,          0.,         1.        ]])

def isclose(x, y, rtol=1.e-5, atol=1.e-8):
    return abs(x-y) <= atol + rtol * abs(y)

# def euler_angles_from_rotation_matrix(R):
#     '''
#     From a paper by Gregory G. Slabaugh (undated),
#     "Computing Euler angles from a rotation matrix

#     return psi(roll), theta(pitch), phi(yaw)
#     '''
#     phi = 0.0
#     if isclose(R[2,0],-1.0):
#         theta = math.pi/2.0
#         psi = math.atan2(R[0,1],R[0,2])
#     elif isclose(R[2,0],1.0):
#         theta = -math.pi/2.0
#         psi = math.atan2(-R[0,1],-R[0,2])
#     else:
#         theta = -math.asin(R[2,0])
#         cos_theta = math.cos(theta)
#         psi = math.atan2(R[2,1]/cos_theta, R[2,2]/cos_theta)
#         phi = math.atan2(R[1,0]/cos_theta, R[0,0]/cos_theta)
#     return psi, theta, phi

# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

# Calculates rotation matrix to euler angles
# The result is the same as MATLAB except the order
# of the euler angles ( x and z are swapped ).
def euler_angles_from_rotation_matrix(R) :

    assert(isRotationMatrix(R))

    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

    singular = sy < 1e-6

    if  not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z])

if __name__ == '__main__':
    rotation_matrix = T_cam_lidar[:,:3]
    print('cam frame to lidar frame')
    print(rotation_matrix)

    r,p,y = euler_angles_from_rotation_matrix(rotation_matrix)
    print('roll:{},pitch:{},yaw:{}'.format(r,p,y))

    x,y,z = T_cam_lidar[0,3],T_cam_lidar[1,3],T_cam_lidar[2,3]
    print('x:{},y:{},z:{}'.format(x,y,z))

    T_lidar_cam = T_cam_lidar.I
    rotation_matrix = T_lidar_cam[:,:3]
    print('lidar frame to cam frame')
    print(rotation_matrix)
    r,p,y = euler_angles_from_rotation_matrix(rotation_matrix)
    print('roll:{},pitch:{},yaw:{}'.format(r,p,y))

    x,y,z = T_lidar_cam[0,3],T_lidar_cam[1,3],T_lidar_cam[2,3]
    print('x:{},y:{},z:{}'.format(x,y,z))
