#!/usr/bin/env python
import rospy, math, yaml, cv2, os
import numpy as np
from tf import TransformListener
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import  PointCloud, Image
from find_keypoints import findKeypoints, loadImagePair
from icp import augmentScene


class StructureFromStereoView(object):

    def __init__(self):
        #initialize attributes
        self.readYAML() 	#create camera attributes
        self.scene = None	#point cloud of scene
        self.dThres = 0.1       #linear movement before performing an update
        self.aThres = math.pi/8 #angular movement before performing an update
        self.lastPose = None    #last view location of robot in odom
        self.currPose = None    #current odom position of robot
        self.imgL = None        #left image
        self.imgR = None        #right image

        # Initialize ROS message to OpenCV converter
        self.bridge = CvBridge() 

        # enable listening for coordinate transforms
        self.tf_listener = TransformListener() 

        #create ros node
        rospy.init_node('sfsv')
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("scene", PointCloud, queue_size=10)
        rospy.Subscriber("camera_left/camera/image_rect_color", Image, self.processRCam)
        rospy.Subscriber("camera_left/camera/image_rect_color", Image, self.processLCam)  
        rospy.Subscriber("odom", Odometry, self.processOdom)
    
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
        #create path to YAMLs
        dir_path = os.path.dirname(os.path.realpath(__file__))
        pathL = os.path.join(dir_path, '../launch/Camera/camera_left_parameters.yaml')
        pathR = os.path.join(dir_path, '../launch/Camera/camera_right_parameters.yaml')

        #load YAML files
        with open(pathL, 'r') as f:
            camLParams = yaml.load(f)        
        with open(pathR, 'r') as f:
            camRParams = yaml.load(f) 

        #extract attributes in proper shape
        self.LK  = np.reshape(np.array(camLParams["camera_matrix"]["data"]), (3,3))   
        self.LProjMat  = np.reshape(np.array(camLParams["projection_matrix"]["data"]), (3,4))        
        self.RK  = np.reshape(np.array(camRParams["camera_matrix"]["data"]), (3,3))   
        self.RProjMat  = np.reshape(np.array(camRParams["projection_matrix"]["data"]), (3,4)) 

    def processOdom(self, msg):
        """odom message callback that stores the robots current pose in self.currPose"""
        self.currPose = msg.pose.pose

    def processLCam(self, msg):
        """left camera image callback that stores image"""
        self.imgL = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def processRCam(self, msg):
        """Right camera image callback that stores image"""
        self.imgR = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    def determineMotion(self):
        """Determines if robot has moved enough to warrent creating another camera view"""

        #initialize scene
        if self.lastPose == None and self.currPose != None:
            self.lastPose = self.currPose
            return True

        if self.currPose == None:
            print("no current pose data")
            return False
        
        #create motion deltas
        aDel = math.fabs(self.lastPose.orientation.z - self.currPose.orientation.z)
        xDel = math.fabs(self.lastPose.position.x - self.currPose.position.x)
        yDel = math.fabs(self.lastPose.position.y - self.currPose.position.y)

        #determine if deltas sufficent
        if aDel > self.aThres or xDel > self.dThres or yDel > self.dThres:
            self.lastPose = self.currPose
            return True            
        
        #if neither first view nor enough motion, return false
        return False

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

        #***placeholder code***
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
    
    def static(self, file_name, file_path):
        """
        Create point cloud scene from two filed stereo images
        Inputs:	file_name (string of base name of file pair)
                file_path (string of path to image directory)
        """
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
        """Main live camera loop that adds views to pt cloud scene when neato moves"""
        while not rospy.is_shutdown:
            #determine if creating new view
            flag = self.determineMotion()
            if flag:
                #create lists of matched points from images
                LMPs,RMPs  = self.extractKeypoints([self.imgL,self.imgR])

                #create point cloud from key points
                view = self.createViewPtCloud(self, LMPs, RMPs)

                #integrate into memory of point cloud scene
                self.integrateView(view)
        
            #publish point cloud
            self.pub.publish(self.scene)
            rospy.sleep(self.rate)

if __name__ == "__main__":
   sfsv = StructureFromStereoView()

   #create point cloud from filed images
   file_name = "backpack"
   file_path = "../images/"
   sfsv.static(file_name, file_path)
