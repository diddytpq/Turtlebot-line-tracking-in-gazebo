#!/usr/bin/env python

import numpy as np
import time
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#rosservice call /gazebo/set_model_state '{model_state: { model_name: turtlebot3_waffle, pose: { position: { x: -1.55, y: 1.915 ,z: 0 }, orientation: {x: -1.72, y: 0.0015, z: 4.225, w: 0.999 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 7.66 } } , reference_frame: world } }'

roslib.load_manifest('turtlebot3_gazebo')

kernel_dilation = cv2.getStructuringElement(cv2.MORPH_RECT,(50,50))

fourcc = cv2.VideoWriter_fourcc(*'DIVX')
out = cv2.VideoWriter("main.mp4", fourcc, 30, (960,540))

class Image_converter:

    def __init__(self):
        #self.image_pub = rospy.Publisher("/camera/rgb/image_raw",Image)
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        self.bridge = CvBridge()
        rospy.init_node('Image_converter', anonymous=True)
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

        self.vel_msg.linear.x = 2
        self.vel_msg.linear.z = 0

    def get_gray(self, img):
        
        self.main_img = img
        self.img_gray = cv2.cvtColor(self.main_img, cv2.COLOR_BGR2GRAY)
        self.img_blur = cv2.GaussianBlur(self.img_gray, (5,5), 0)
        _, self.binary = cv2.threshold(self.img_blur, 127, 255, cv2.THRESH_BINARY)

        return self.binary

    def get_roi(self, img):

        self.mask = np.zeros_like(img)

        roi_range=np.array([[(100, 100),(860, 100),(690, 540), (300, 540)]],dtype=np.int32)
        cv2.fillPoly(self.mask,roi_range,255)

        return cv2.bitwise_and(img, self.mask)

    def get_contour_center(self, img):

        self.contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
        max_index = 0

        if len(self.contours) == 0:
            return False

        if len(self.contours) > 1 :
            area_list = []

            for cnt in self.contours:
                M = cv2.moments(cnt)
                area = cv2.contourArea(cnt)

                area_list.append(area)

            max_index = np.argmax(area_list)


        M = cv2.moments(self.contours[max_index])
        
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        cv2.line(self.main_img, (cx,cy), (cx,cy) ,(0,255,0),5)
                    
        cv2.drawContours(self.main_img, self.contours, -1, (0,255,0), 3)

        return (cx, cy)




    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :


            #print(cv_image.shape)

            self.main_img = cv2.resize(cv_image,(0,0),fx = 0.5, fy = 0.5, interpolation = cv2.INTER_AREA)

            self.gray_img = self.get_gray(self.main_img)
            self.roi_img = self.get_roi(self.gray_img)

            self.fgmask_dila = cv2.dilate(self.roi_img,kernel_dilation,iterations = 1)

            #self.lines=cv2.HoughLinesP(self.img_canny,1,np.pi/180,25,minLineLength=1,maxLineGap=210)
            self.center_point = self.get_contour_center(self.fgmask_dila)

            if self.center_point:

                self.error = (self.center_point[0]-480, self.center_point[1])
                print(self.error)

                
                self.vel_msg.linear.x = 0.35

                self.vel_msg.angular.z = -(self.error[0] / 60)



            
            

            self.velocity_publisher.publish(self.vel_msg)

            cv2.line(self.main_img, (480,0),(480, 640),(0,0,255),5)

            #cv2.imshow("Image window", self.fgmask_dila)
            cv2.imshow("main", self.main_img)

            out.write(self.main_img)





            key = cv2.waitKey(3)

            if key == 27 : 
                                
                self.vel_msg.linear.x = 0

                self.vel_msg.angular.z = 0

                self.velocity_publisher.publish(self.vel_msg)
                cv2.destroyAllWindows()
                
                return 0

        #try:
        #    self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        #except CvBridgeError as e:
        #    print(e)

def main(args):

    ic = Image_converter()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)