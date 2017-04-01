import cv2
import numpy as np
import rospy

def LinearLSTTriangulation(u1, P1, u2, P2):
    matrixA = [u1[0]*P1[2,0]-P1[0,0], u1[0]*P1[2,1]-P1[0,1], u1[0]*P1[2,2]-P1[0,2],
               u1[1]*P1[2,0]-P1[1,0], u1[1]*P1[2,1]-P1[1,1], u1[1]*P1[2,2]-P1[1,2],
               u2[0]*P2[2,0]-P2[0,0], u2[0]*P2[2,1]-P2[0,1], u2[0]*P2[2,2]-P2[0,2],
               u2[1]*P2[2,0]-P2[1,0], u2[1]*P2[2,1]-P2[1,1], u2[1]*P2[2,2]-P2[1,2]]
    matrixA = np.reshape(np.array(matrixA), [4,3])

    matrixB = [-u1[0]*P1[2,3]-P1[0,3], -u1[1]*P1[2,3]-P1[1,3], -u1[0]*P2[2,3]-P2[0,3],-u1[1]*P2[2,3]-P2[1,3]]
    matrixB = np.reshape(np.array(matrixB), (4,1))

    return np.linalg.lstsq(matrixA, matrixB)[0]

def triangulate_opencv(pt_set1, pt_set2, P, P1):
    my_points = cv2.triangulatePoints(P,P1,pt_set1.T,pt_set2.T)
    projected_points_1 = P.dot(my_points)

    # convert to inhomogeneous coordinates
    for i in range(projected_points_1.shape[1]):
        projected_points_1[0,i] /= projected_points_1[2,i]
        projected_points_1[1,i] /= projected_points_1[2,i]
        projected_points_1[2,i] /= projected_points_1[2,i]

    projected_points_2 = P1.dot(my_points)
    # convert to inhomogeneous coordinates
    for i in range(projected_points_2.shape[1]):
        projected_points_2[0,i] /= projected_points_2[2,i]
        projected_points_2[1,i] /= projected_points_2[2,i]
        projected_points_2[2,i] /= projected_points_2[2,i]

    # convert to inhomogeneous coordinates
    for i in range(projected_points_2.shape[1]):
        my_points[0,i] /= my_points[3,i]
        my_points[1,i] /= my_points[3,i]
        my_points[2,i] /= my_points[3,i]
        my_points[3,i] /= my_points[3,i]

    return my_points.T

def triangulate(points1, points2, P1, P2, K1, K2):
    K1inv = np.linalg.inv(K1)
    K2inv = np.linalg.inv(K2)
    for i, point1 in enumerate(points1):
        point1 = np.array([point1[0],point1[1],1.0])
        point2 = points2[i]
        point2 = np.array([point2[0],point2[1],1.0])

        um1 = K1inv*point1
        um2 = K2inv*point2

        matrixX = LinearLSTTriangulation(um1, P1, um2, P2)

k1 = np.array([[582.7775802992247, 0.0, 311.6859398711437],
    [0.0, 580.8376807520034, 256.3266732337835],
    [0.0, 0.0, 1.0]])
k2 = np.array([[582.7775802992247, 0.0, 311.6859398711437],
    [0.0, 580.8376807520034, 242.004997],
    [0.0, 0.0, 1.0]])

p1 = np.array([[494.6373026601465, 0.0, 272.9442329406738, -50.45653648390667],
      [0.0, 494.6373026601465, 248.702205657959, 0.0],
      [0.0, 0.0, 1.0, 0.0]])

p2 = np.array([[494.6373026601465, 0.0, 272.9442329406738, 0.0],
      [0.0, 494.6373026601465, 248.702205657959, 0.0],
      [0.0, 0.0, 1.0, 0.0]])

if __name__ == '__main__':
    K1inv = np.linalg.inv(k1)
    K2inv = np.linalg.inv(k2)
    point1 = [486.0,101.0]
    point2 = [314.0,150.0]
    point1 = np.array([point1[0],point1[1]])
    point2 = np.array([point2[0],point2[1]])

    um1 = K1inv*np.array([point1[0],point1[1],1.0])
    um2 = K2inv*np.array([point2[0],point2[1],1.0])
    print(triangulate_opencv(point1, point2, p1, p2))
    print(LinearLSTTriangulation(um1[0,:],p2,um2[0,:],p1))
