#!/usr/bin/python 
# -- coding: utf-8 --
import cv2
import sys
import numpy as np
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node("cv_bridge", anonymous=True)
pospub = rospy.Publisher('target_pos', Pose, queue_size=10)

class CameraHandler:
    def __init__(self):
        self.img = np.zeros((1000, 1000))
        self.bridge = CvBridge()
        rospy.Subscriber("camera/image_raw", Image, self.callback)
        rospy.wait_for_message("camera/image_raw", Image)


    def callback(self, msg):
        try:
            self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        except CvBridgeError, e:
            print("Frame Dropped: ", e)


    def fetchImg(self):
        return self.img


    def showImg(self):
        cv2.imshow("Frontal Camera", self.img)


handler = CameraHandler()


while not rospy.is_shutdown():
    #get the image
    frame = handler.fetchImg()

    #Converting the frames to HSV, because with this color
    #space also shows us the value (or intensity) of each
    #color, making it easier for us to find the red cylinder
    redfilter = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #defining the color limits for our mask
    Lred = (0,149,140)
    Dred = (0,255,177)
    cylinder_mask = cv2.inRange(redfilter, Lred, Dred)

    #applying the mask to our feed
    cylinder = cv2.bitwise_and(frame, frame, mask = cylinder_mask)

    # making a contour around the cylinder
    im, contours, hierarchy = cv2.findContours(cylinder_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # if we have at least one contour in the color of the cylinder, we find its area and choose the largest one
    if len(contours)>0:
        contour = max(contours,key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area > 250:
            x, y, w, h = cv2.boundingRect(contour)	# gets the x, y coordinates of the shape and its width and height
            img=cv2.circle(frame, ((2*x+w)/2,(2*y+h)/2),5,(0,0,255),1) #draws shapes on top of the image
            img=cv2.line(frame, ((frame.shape[0]), (frame.shape[1])),((2*x+w)/2,(2*y+h)/2),(0,255,0),2)

            #creates variables with the position of the shape relative to the center of the feed and publishes it
            targetdx = ((2*x+w)/2) - frame.shape[0]
            targetdy = ((2*y+h)/2) - (frame.shape[1])
            target = Pose()
            target.position.x = targetdx
            target.position.y = targetdy
            target.position.z = 0
            pospub.publish(target)
            #print targetdx
    else: #if we don't find the cylinder, we send 0's
        target = Pose()
        target.position.x = 0
        target.position.y = 0
        target.position.z = 0
        pospub.publish(target)

    # when uncommented, these lines display the images we obtained
    #cv2.imshow("Original", frame)
    #cv2.imshow("Filtered", cylinder)

    #break the loop if we press 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

#when we leave the loop, stop the capture and close all windows
cv2.destroyAllWindows()
