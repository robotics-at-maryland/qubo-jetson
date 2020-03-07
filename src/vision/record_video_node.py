import cv2
import rospy
from sensor_msgs import Image
import numpy as np

def callback(data):
	img = np.reshape(data.data, (data.height,data.width))
	cv2.imshow("image", img)

def listener():
	rospy.init_node("listener", anonymous=True)
	rospy.Subscriber("camera/image_raw")
	rospy.spin()

if __name__ == "__main__":
	listener()
