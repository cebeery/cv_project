import cv2
import pickle
import numpy as np
import sys
import rospkg
import rospy

# Things that neet to be accessed by mouse_event
im1_pts = []
im2_pts = []
im1_size = ()
im2_size = ()

def mouse_event(event,x,y,flag,im):
    """ Catch mouse events so we can draw epipolar lines when clicked """
    if event == cv2.EVENT_FLAG_LBUTTON:
        if x < im1_size[1]:
            im1_pts.append((x,y))
        else:
            im2_pts.append((x - im1_size[1],y))

        print(im1_pts)
        print(im2_pts)

def save_points():
    points = (im1_pts, im2_pts)
    pickle.dump(points, open( "points.p", "wb" ))

if __name__ == '__main__':
    # Access the side by side images
    rospack = rospkg.RosPack()
    im1_file = rospack.get_path('cv_project') + '/images/trash_cans_left.png'
    im2_file = rospack.get_path('cv_project') + '/images/trash_cans_right.png'

    im1 = cv2.imread(im1_file)
    im2 = cv2.imread(im2_file)
    im1_size = im1.shape
    im2_size = im2.shape
    print (im1_size)

    # Combine into one image for display purposes
    im = np.array(np.hstack((im1,im2)))

    cv2.imshow("MYWIN",im)
    cv2.setMouseCallback("MYWIN",mouse_event,im)
    rospy.on_shutdown(save_points)
    while not rospy.is_shutdown():
        cv2.imshow("MYWIN",im)
        cv2.waitKey(50)
    cv2.destroyAllWindows()
