#!/usr/bin/env python

import rospy
import numpy as np
from sklearn.neighbors import NearestNeighbors
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud
import utils
import icp
from sklearn.neighbors import NearestNeighbors

def subXYTPose(current, previous):
	return tuple([current[i]-previous[i] for i in range(len(current))])

def addXYTPose(current, previous):
	return tuple([current[i]+previous[i] for i in range(len(current))])

class ICPWrapper(object):
	def __init__(self, matchConsiderationThreshold=0.05, samePointRadius=0.001):
		self.estimatedPose = None
		self.lastOdom = None
		self.scenePC = PointCloud(header='odom')
		self.sceneTree = NearestNeighbors(n_neighbors=1, radius=matchConsiderationThreshold)
		self.matchConsiderationThreshold = matchConsiderationThreshold
		self.samePointRadius = samePointRadius

		rospy.Subscriber('/odom', Odometry, self.estimatePose)
		rospy.Subscriber('/singleViewPC', PointCloud, self.executeICP)
		self.publisher = rospy.Publisher('/scenePC', PointCloud, queue_size=10)

	def estimatePose(self, odom):
		newOdom = utils.poseToXYTheta(odom.pose.pose)

		# If no pose has yet been recorded, use the first one received
		if self.estimatedPose is None or self.lastOdom is None:
			self.estimatedPose = newOdom
			self.lastOdom = newOdom
			return

		# Otherwise, take the delta from the last odom and add it to
		# the updated pose.
		deltaOdom = subXYTPose(self.lastOdom, self.newOdom)
		estimatedPose = addXYTPose(self.estimatedPose, deltaOdom)
		self.lastOdom = newOdom

	def executeICP(self, newPC):

		# TODO: Estimate x & z offset of cameras (or else rotation may interfere w/ scene)
		# TODO: re-implement using a TF transform to the odom frame
		# Compensate for the different coordinate frames of the cameras
		if newPC.header.frame == 'right_cam':
			newPC.points = [Point(p.x, p.y+0.05, p.z) for p in newPC.points]
		elif newPC.header.frame == 'left_cam':
			newPC.points = [Point(p.x, p.y-0.05, p.z) for p in newPC.points]
		else:
			raise ValueError(
				'The point cloud frame passed to icp_guess was neither ' /
				'"right_cam" nor "left_cam"'
			)

		# If this is the first view we've seen, use it as the basis for the scene
		if self.scenePC is None:
			self.scenePC = newPC
			self.sceneTree.fit(self.scenePC.points)
			return

		# If we don't have any odometry data, wait until we get some
		if self.estimatedPose is None:
			return

		# Select subset of scene points that most closely match the view,
		# but discards any potential pairs that are off by a certain threshold
		sceneIndex, distances = self.sceneTree.kneighbors(newPC.points)

		sceneICPPoints, newICPPoints = ([], [])
		for i, d in enumerate(distances):
			if d <= self.matchConsiderationThreshold:
				continue

			sceneICPPoints.append(self.scenePC[sceneIndex[i]])
			newICPPoints.append(newPC.points[i])

		# Apply ICP
		transformM, errors = icp.icp(np.array(newICPPoints), np.array(sceneICPPoints))
		transformedPC = np.dot(transformM, np.array(newICPPoints))

		# Only include new (non-overlapped) points to avoid cluttering the scene
		_, distances = self.sceneTree.kneighbors(transformedPC)
		validatedPoints = [transformedPC[i] for i, d in enumerate(distances) if d >= self.samePointRadius]

		# TODO: Determine the camera perspective and update the estimated transform

		# Add the newly validated points to the scene and rebuild the scene tree
		self.scenePC.extend(validatedPoints)
		self.sceneTree.fit(self.scenePC.points)

		# Publish updated scene
		self.publisher.publish(self.scenePC)

	def run(self):
		r = rospy.Rate(5)

		while not rospy.is_shutdown():
			r.sleep()

if __name__ == "__main__":
	node = ICPWrapper
	node.run()