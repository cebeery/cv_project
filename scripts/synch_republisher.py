import rospy
from sensor_msgs.msg import Image
from std_msgs import Header


class Republisher(object):

    def __init__(self):
        """ Initialize the street sign reocgnizer """
        rospy.init_node('camera_republisher')
        self.camera1_subscriber = rospy.Subscriber('camera', Image, set_image_1)
        self.camera2_subscriber = rospy.Subscriber('camera2', Image, set_image_2)
        self.camera1_publisher = rospy.Publisher('/camera/camera1_synced', Image, queue_size = 5)
        self.camera2_publisher = rospy.Publisher('/camera/camera2_synced', Image, queue_size = 5)
        self.camera1_image = None                 # the latest image from the camera
        self.camera2_image = None

    def set_image_1(self, img):
        self.camera1_image = img

    def set_image_2(self, img):
        self.camera2_image = img

    def stop(self):
        pass

    def run(self):
        r = rospy.Rate(30)
        rospy.on_shutdown(self.stop)
        while not rospy.is_shutdown():
            header = Header()
            header.time = rospy.Time.now()
            self.camera1_image.header = header
            self.camera2_image.header = header
            self.camera1_publisher.publish(self.camera1_image)
            self.camera2_publisher.publish(self.camera2_image)
            r.sleep()

if __name__ == '__main__':
    syncher = Republisher()
    syncher.run()
