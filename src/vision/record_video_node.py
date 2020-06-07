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
		self.actions = []
		if args["output"]:
			self.video_writer = cv2.VideoWriter(args["output"],
							    cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),
							    args["fps"],
							    (args["width"], args["height"]))
			self.actions.append(lambda frame: self.video_writer.write(frame))

		if args["display"]:
			def display_delay(frame):
				cv2.imshow("image")
				cv2.waitKey(1)
			self.actions.append(display_delay)

		# create lambdas for configurable callback behavior (all must take one input, the frame)


	def callback(self,data):
		try:
			frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
			for action in self.actions:
				action(frame)
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
	ap.add_argument("-w", "--width", type=int, default=1280,
			help="Width in pixels of camera stream")
	ap.add_argument("-l", "--height", type=int, default=1024,
			help="Height in pixels of camera stream")

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
