#!/usr/bin/env python
import cv2
import os
import draw_matches as dm

def findKeypoints(imgPair, visualize=False, savePath=None):

	if visualize:
		showImgPair(imgPair, savePath=savePath)

	kpPair = detectKeypointPair(imgPair)

	if visualize:
		showKeypointPair(imgPair, kpPair, savePath=savePath)

	matches = matchKeypointsByRatio(kpPair)

	if visualize:
		showMatchPair(imgPair, kpPair, matches, savePath=savePath)

	leftImg, rightImg = imgPair
	leftKps, leftDes, rightKps, rightDes = kpPair

	return {
		"rightImg": rightImg,
		"rightKps": rightKps,
		"rightDes": rightDes,
		"leftImg": leftImg,
		"leftKps": leftKps,
		"leftDes": leftDes,
		"matches": matches
	}

def loadImage(imgName, path="../images/"):
	return cv2.imread(os.path.join(path, imgName), 0)

def loadImagePair(baseName, path="../images/", ext=".png"):
	left = loadImage(baseName + "_left" + ext, path);
	right = loadImage(baseName + "_right" + ext, path);
	return (left, right)

def showImgPair(imgPair, pause=True, savePath=None):
	left, right = imgPair
	cv2.imshow('left', left)
	cv2.imshow('right', right)

	if pause:
		cv2.waitKey(0)
		cv2.destroyWindow('left')
		cv2.destroyWindow('right')

	if savePath:
		cv2.imwrite(os.path.join(savePath, 'left_img.png'), left)
		cv2.imwrite(os.path.join(savePath, 'right_img.png'), right)

def detectKeypoints(image):
	detector = cv2.ORB(1000, 1.2)
	kp, des = detector.detectAndCompute(image, None)
	return (kp, des)

def detectKeypointPair(imgPair):
	left, right = imgPair
	leftKp, leftDes = detectKeypoints(left)
	rightKp, rightDes = detectKeypoints(right)
	return (leftKp, leftDes, rightKp, rightDes)

def showKeypointPair(imgPair, keypointPair, pause=True, savePath=None):
	leftImg, rightImg = imgPair
	leftKps, _, rightKps, _ = keypointPair

	leftKpImg = cv2.drawKeypoints(leftImg, leftKps, None, color=(0, 0, 255))
	rightKpImg = cv2.drawKeypoints(rightImg, rightKps, None, color=(0, 0, 255))

	cv2.imshow('left_kps', leftKpImg)
	cv2.imshow('right_kps', rightKpImg)

	if pause:
		cv2.waitKey(0)
		cv2.destroyWindow('left_kps')
		cv2.destroyWindow('right_kps')

	if savePath:
		cv2.imwrite(os.path.join(savePath, 'left_kps.png'), leftKpImg)
		cv2.imwrite(os.path.join(savePath, 'right_kps.png'), rightKpImg)

def matchKeypointsByRatio(keypointPair, ratio=0.75):
	"""
	Picks any matches that have a distance (badness)
	lower than <ratio>.
	"""
	_, leftDes, _, rightDes = keypointPair

	bf = cv2.BFMatcher()
	matches = bf.knnMatch(leftDes, rightDes, k=2)
	return [m for (m,n) in matches if m.distance < ratio*n.distance]

def matchKeypointsByQuota(keypointPair, quota=10):
	"""
	Picks the top <quota> matches between the two images.
	"""
	_, left_des, _, right_des = keypointPair

	bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
	matches = bf.match(left_des, right_des)
	return sorted(matches, key=lambda val: val.distance)[:quota]

def showMatchPair(imgPair, keypointPair, matches, pause=True, savePath=None):
	leftImg, rightImg = imgPair
	leftKps, _, rightKps, _ = keypointPair

	matchImg = dm.drawMatches(
		leftImg,
		leftKps,
		rightImg,
		rightKps,
		matches,
		pause
	)

	if savePath:
		cv2.imwrite(os.path.join(savePath, 'matches.png'), matchImg)

if __name__ == "__main__":
        dir_path = os.path.dirname(os.path.realpath(__file__))
	TEST_FILEPATH = os.path.join(dir_path, "../images/")
	TEST_FILE = "backpack"
        imgPair = loadImagePair(TEST_FILE, TEST_FILEPATH)

	kpMatchData = findKeypoints(imgPair, visualize=True, savePath=('../docs/'+TEST_FILE+'_'))
	print kpMatchData.keys()
