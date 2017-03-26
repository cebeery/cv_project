#!/usr/bin/env python
import cv2
import os
import draw_matches

def loadImage(img_name, path="../images/"):
	return cv2.imread(os.path.join(path, img_name), 0)

def loadImagePair(base_name, path="../images/", ext=".png"):
	left = loadImage(base_name + "_left" + ext, path);
	right = loadImage(base_name + "_right" + ext, path);
	return (left, right)

def showImgPair(imgPair, pause=True):
	left, right = imgPair
	cv2.imshow('left', left)
	cv2.imshow('right', right)

	if pause:
		cv2.waitKey(0)
		cv2.destroyWindow('left')
		cv2.destroyWindow('right')

def detectKeypoints(image):
	detector = cv2.ORB(1000, 1.2)
	kp, des = detector.detectAndCompute(image, None)
	return (kp, des)

def detectKeypointPair(imgPair):
	left, right = imgPair
	left_kp, left_des = detectKeypoints(left)
	right_kp, right_des = detectKeypoints(right)
	return (left_kp, left_des, right_kp, right_des)

def showKeypointPair(imgPair, keypointPair, pause=True):
	left_img, right_img = imgPair
	left_kps, _, right_kps, _ = keypointPair

	left_kpImg = cv2.drawKeypoints(left_img, left_kps, None, color=(0, 0, 255))
	right_kpImg = cv2.drawKeypoints(right_img, right_kps, None, color=(0, 0, 255))

	cv2.imshow('left_kps', left_kpImg)
	cv2.imshow('right_kps', right_kpImg)

	if pause:
		cv2.waitKey(0)
		cv2.destroyWindow('left_kps')
		cv2.destroyWindow('right_kps')

def matchKeypointsByRatio(keypointPair, ratio=0.75):
	"""
	Picks any matches that have a distance (badness)
	lower than <ratio>.
	"""
	_, left_des, _, right_des = keypointPair

	bf = cv2.BFMatcher()
	matches = bf.knnMatch(left_des, right_des, k=2)
	return [m for (m,n) in matches if m.distance < ratio*n.distance]

def matchKeypointsByQuota(keypointPair, quota=10):
	"""
	Picks the top <quota> matches between the two images.
	"""
	_, left_des, _, right_des = keypointPair

	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	matches = bf.match(left_des, right_des)
	return sorted(matches, key=lambda val: val.distance)[:quota]

def showMatchPair(imgPair, keypointPair, matches, pause=True):
	left_img, right_img = imgPair
	left_kps, _, right_kps, _ = keypointPair

	matchImg = drawmatches.drawMatches(
		left_img,
		left_kps,
		right_img,
		right_kps,
		matches,
		pause
	)

if __name__ == "__main__":
	TEST_FILEPATH = "../images/"
	TEST_FILE = "backpack"

	imgPair = loadImagePair(TEST_FILE, TEST_FILEPATH)
	showImgPair(imgPair)

	kpPair = detectKeypointPair(imgPair)
	showKeypointPair(imgPair, kpPair)

	matches = matchKeypointsByQuota(kpPair)
	showMatchPair(imgPair, kpPair, matches)