#!/usr/bin/env python
import rospy, math, yaml
import numpy as np
from tf import TransformListener
#from tf.transformations import euler_from_quaternion, rotation_matrix, quaternion_from_matrix, quaternion_from_euler
from geometry_msgs.msg import PointCloud, Point
from find_keypoints import findKeypoints, loadImagePair
from icp import augmentScene

class StructureFromStereoView(object):

    def __init__(self):

        #initialize attributes
        self.readYAML() 	#create camera attributes
        self.scene = None	#point cloud of scene
        self.dThres = 0.2       #linear movement before performing an update
        self.aThres = math.pi/6 #angular movement before performing an update

        #create ros node
        rospy.init_node('sfsv')
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("scene", PointCloud, queue_size=10)

        # enable listening for coordinate transforms
        self.tf_listener = TransformListener() 
     
    def readYAML(self):
        """
        Set camera calibration and projection matrixes class attributes 
        Requires: specific YAML file name and path 
        Sets (as numpy arrays):	
		LK (3x3 left camera calibration matrix)
        	RK (3x3 right camera calibration matrix)
		LProjMat (3x4 left camera projection matrix)
		RProjMat (3x4 right camera projection matrix)
        """
        
        #load YAML files
        with open('../launch/Camera/camera_left_parameters.yaml', 'r') as f:
            camLParams = yaml.load(f)        
        with open('../launch/Camera/camera_right_parameters.yaml', 'r') as f:
            camRParams = yaml.load(f) 

        #extract attributes in proper shape
        self.LK  = np.reshape(np.array(camLParams["camera_matrix"]["data"]), (3,3))   
        self.LProjMat  = np.reshape(np.array(camLParams["projection_matrix"]["data"]), (3,4))        
        self.RK  = np.reshape(np.array(camRParams["camera_matrix"]["data"]), (3,3))   
        self.RProjMat  = np.reshape(np.array(camRParams["projection_matrix"]["data"]), (3,4)) 
 
    def extractKeypoints(self, imgPair, visualize=False):
        """
        Create lists of points from each image whose lined up indices indicate matches
        Input: 
        	imgPair (list of two assumed stereo images [L,R])
                visualize (boolean, to visualize matches, blocking thread)
        Calls: 	find_keypoints.py
        Outputs: 
               	leftMatched (trimmed and ordered 1D np array of left image points by match)
               	rightMatched (trimmed and ordered 1D np array of right image points by match)
        """

        #generate matches and keypoints 
        viewData = findKeypoints(imgPair, visualize=True)

        #extract data
        leftKps = viewData["leftKps"]
        rightKps = viewData["rightKps"]
        matches = viewData["matches"]

        #created ordered list of matched points
        leftMatched = []
        rightMatched = []
        for i in range(0, len(matches)):
            #queryIdx is the left image; trainIdx is the right image 
            leftMatched.append(leftKps[matches[i].queryIdx].pt)
            rightMatched.append(rightKps[matches[i].trainIdx].pt)

        #convert list to numpy array
        leftMatched =  np.array(leftMatched)
        rightMatched = np.array(rightMatched)

        return leftMatched, rightMatched


    def verifyProjectionMat(self):
        """Calculates expected projection matrices from two images and compares to imported matrices"""
        #TODO
        pass

    def createViewPtCloud(self):
        """
        Create point cloud of current view from matched points
        Inputs:
		LMPs (ordered 1D array of left image matched points)
               	RMPs (ordered 1D array of right image matched points)
        Output: view (point cloud of current stereo view)
        """
        #TODO 
	#change to triangulate call
        
        #view = triangulate(LMPs, RMPs, self.LK, self.RK, self.LProjMat, self.RProjMat)

        #placeholder code
        view = PointCloud()
        view.frame_id="view_zero"
        view.points = [ Point(p[0], p[1], 0) for p in [
                      (0, 0),
                      (1, 1),
                      (-3, -2),
                      ]]

        #transform current view point cloud from view_zero frame to odom frame
        view_odom = self.tf_listener.PointCloud("odom", view) 

        return view_odom

    def integrateView(self, view):
        """
        Updates scene point cloud with current view
        Input:	view (point cloud of current view)
        """        
        #perform ICP to add current view point cloud to cummulative scene 
        if self.scene == None:
            self.scene = view #set initial scene point cloud if first view
        else:
            self.scene,_ = augmentScene(view, self.scene)

    def refineScene(self):
        """Perform bundle adjustment refinement on scene"""
        #TODO
        pass
    
    def static(self, file_name, file_path)
        """add to pointcloud scene from two filed images"""
        #TODO
        #implement multiple view via remembered odoms

        #create matched point lists
        imgPair = loadImagePair(file_name,file_path)
        LMPs,RMPs = self.extractKeypoints(imgPair, visualize=True)

        #create point cloud from key points
        view = self.createViewPtCloud(self, LMPs, RMPs)

        #publish point cloud
        while not rospy.is_shutdown:
            self.pub.publish(view)
            rospy.sleep(self.rate)

    def live(self):
        #TODO
        #implement cameras and odom limiters; intergrate other teammates libraries

        while not rospy.is_shutdown:
            #create lists of matched points from images
            LMPs,RMPs  = self.extractKeypoints(imgPair)

            #create point cloud from key points
            view = self.createViewPtCloud(self, LMPs, RMPs)

            #integrate into memory of point cloud scene
            self.integrateView(view):
        
            #publish point cloud
            self.pub.publish(self.scene)
            rospy.sleep(self.rate)

if __name__ == __main__:
   sfsv = StructureFromStereoView()

   #create point cloud from filed images
   file_name = "backpack"
   file_path = "../images/"
   sfsv.static(file_name, file_path)
