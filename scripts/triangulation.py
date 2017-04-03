import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import std_msgs


def triangulate_opencv(pt_set1, pt_set2, P, P1):
    pt_set1 = pt_set1.astype(float)
    pt_set2 = pt_set2.astype(float)
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

    return create_pointcloud(my_points.T)

def create_pointcloud(pts):
    depths = PointCloud()
    depths.header = std_msgs.msg.Header()
    depths.header.stamp = rospy.Time.now()
    depths.header.frame_id = "view_0"
    depths.points = [None] * len(pts)
    for p in xrange(len(pts)):
        x = pts[p,0]
        y = pts[p,1]
        z = pts[p,2]
        depths.points[p] = Point(x, y, z)
    return depths

if __name__ == '__main__':
    rospy.init_node('triangulator')
    p1 = np.array([[494.6373026601465, 0.0, 272.9442329406738, -50.45653648390667],
          [0.0, 494.6373026601465, 248.702205657959, 0.0],
          [0.0, 0.0, 1.0, 0.0]])

    p2 = np.array([[494.6373026601465, 0.0, 272.9442329406738, 0.0],
          [0.0, 494.6373026601465, 248.702205657959, 0.0],
          [0.0, 0.0, 1.0, 0.0]])

    points1 = np.array([(376, 91), (361, 88), (485, 101), (459, 194), (380, 173)])
    points2 = np.array([(212, 141), (197, 140), (313, 148), (297, 237), (221, 219)])
    print(triangulate_opencv(points1, points2, p1, p2))
