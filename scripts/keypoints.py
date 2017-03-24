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
	# Create SIFT detector
	detector = cv2.SIFT()
	kp, des = detector.detectAndCompute(image, None)
	return (kp, des)

def detectKeypointPair(imgPair):
	left, right = imgPair
	left_kp, left_des = detectKeypoints(left)
	right_kp, right_des = detectKeypoints(right)
	return (left_kp, left_des, right_kp, right_des)

def matchKeypoints(keypointPair):
	_, left_des, _, right_des = keypointPair

	# Init/use brute force matcher,
	# with K-nearest-neighbors alg used to cluster
	# key-points into pairs (k=2)
	bf = cv2.BFMatcher(crossCheck=True)
	matches = bf.match(left_des, right_des)

	# Apply ratio test
	# good_matches = [m for (m,n) in matches if m.distance < 0.75*n.distance]

	return matches

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
