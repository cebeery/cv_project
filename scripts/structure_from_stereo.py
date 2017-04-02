#!/usr/bin/env python
import yaml
import numpy as np
import findKeypoints, loadImagePair

class SFSV(object):

    def __init__(self):
        self.readYAML()
        print self.LProjMat

 
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
 
    def extractKeypoints(self, imgPair):
        """
        Extracts matches and keypoints from images
        Input: imgPair (list of two assumed stereo images [L,R])
        Calls: find_keypoints.py
        Outputs: 
               LP (trimmed and ordered list of left image keypoints by match)
               RP (trimmed and ordered list of right image keypoints by match)
        """

        #TODO

	TEST_FILEPATH = "../images/"
	TEST_FILE = "backpack"
	imgPair = loadImagePair(TEST_FILE, TEST_FILEPATH)
        viewData = findKeypoints(imgPair)

        rightKps = viewData["rightKps"]
        leftKps = viewData["leftKps"]
        matches = viewData["matches"]

        return LP, RP


    def verifyProjectionMat(self):
        """Calculates expected projection matrices from two images and compares to imported matrices"""
        #TODO

    def createViewPtCloud(self):
        """Create point cloud of current view"""
        #TODO

    def camera2odom(self):
        """transform point cloud in camera frame to odom frame"""
        #TODO

    def integrateView(self):
        """Perform ICP to add view point cloud to scene point cloud"""
        #TODO

    def refineScene(self):
        """Perform bundle adjustment refinement on scene"""
        #TODO
        pass

    def main(self):
        TEST_FILEPATH = "../images/"
	TEST_FILE = "backpack"
	imgPair = loadImagePair(TEST_FILE, TEST_FILEPATH)
        LP,RP = self.extractKeypoints(imgPair)

testObj = SFSV()
