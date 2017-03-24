#!/usr/bin/env python
import cv2
import os
import drawmatches

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

def matchKeypoints(keypointPair, nMatches=10):
	_, left_des, _, right_des = keypointPair

	# Get 10 best matches
	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	matches = bf.match(left_des, right_des)
	return sorted(matches, key=lambda val: val.distance)[:nMatches]

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
	TEST_FILE = "trash_cans"


	imgPair = loadImagePair(TEST_FILE, TEST_FILEPATH)

	# showImgPair(imgPair)

	kpPair = detectKeypointPair(imgPair)
	matches = matchKeypoints(kpPair)

	showMatchPair(imgPair, kpPair, matches)
