#!/usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud
import std_msgs


def triangulate(left_pts, right_pts, P, P1):
    """ Takes in two sets of points, and two camera
        projection matrices, to create a depth cloud

        args:
            left_pts: list of points (x,y pairs) from left image
            right_pts: list of points (x,y pairs) from right image
            P: camera projection for left image
            P1: camera projection for right image

        returns:
            rospy pointcloud of points with x,y,z coordinates
    """
    left_pts = np.array(left_pts).astype(float)
    right_pts = np.array(right_pts).astype(float)
    my_points = cv2.triangulatePoints(P,P1,left_pts.T,right_pts.T)

    # convert to inhomogeneous coordinates
    for i in range(my_points.shape[1]):
        my_points[0,i] /= my_points[3,i]
        my_points[1,i] /= my_points[3,i]
        my_points[2,i] /= my_points[3,i]
        my_points[3,i] /= my_points[3,i]

    return create_pointcloud(my_points.T)

def create_pointcloud(pts):
    """ Takes in a numpy array of points, and
        turns it into a ros PointCloud

        args:
            pts: numpy array of 4d inhomogonozed points

        returns:
            a ros PointCloud made of 3d versions of points
    """
    depths = PointCloud()
    depths.header = std_msgs.msg.Header()
    depths.header.stamp = rospy.Time.now()
    depths.header.frame_id = "view_zero"
    depths.points = [None] * len(pts)
    for p in xrange(len(pts)):
        #Giving point the same orientation as the robot
        y = pts[p,0]
        z = - pts[p,1] #y in images is down
        x = pts[p,2]
        depths.points[p] = Point(x, y, z)
    return depths

if __name__ == '__main__':
    #An example of triangulation.
    rospy.init_node('triangulator')

    #Camera matrices that came with the images
    #Used to make point arrays later
    p1 = np.array([[651.2604929444119, 0, 347.6628112792969, 0],
            [ 0, 651.2604929444119, 243.5513744354248, 0],
            [0, 0, 1, 0]])

    p2 = np.array([[651.2604929444119, 0, 347.6628112792969, -65.47091473431786],
            [0, 651.2604929444119, 243.5513744354248, 0],
            [0, 0, 1, 0]])

    #Matching keypoints found manually in two different images
    points1 = [(120, 283), (81, 178), (446, 254), (443, 180), (425, 101), (417, 23), (399, 18), (415, 101)]
    points2 = [(45, 287), (8, 179), (389, 254), (387, 179), (367, 100), (359, 21), (337, 16), (358, 102)]
    print(triangulate(points1, points2, p1, p2))
