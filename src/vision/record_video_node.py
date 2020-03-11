#!/usr/bin/python

import cv2
import rospy
from sensor_msgs.msg import Image
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import argparse

class image_converter:

	def __init__(self,args):
		self.image_sub = rospy.Subscriber("camera/image_color",Image,self.callback)
		self.bridge = CvBridge()
	
	def callback(self,data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv2.resize(cv_image, (600, 800))
			cv2.imshow("image", cv_image)
			cv2.waitKey(1)
		except CvBridgeError as e:
			print(e)	

def main():

	ap = argparse.ArgumentParser()
	ap.add_argument("-d", "--display", type=bool, default=True,
		help="Toggles display of video")
	ap.add_argument("-o", "--output", default=None,
		help="path to output video file")
	ap.add_argument("-f", "--fps", type=int, default=30,
		help="FPS of output video")

	args = vars(ap.parse_args())


	ic = image_converter(args)
	rospy.init_node("listener", anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()
