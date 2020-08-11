#!/usr/bin/env python

import rospy
import sys
import cv2
#import cv2.cv as cv
from kalman_loc_qr.msg import ArucoMasks as Landmarks
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32MultiArray, Header
from cv_bridge import CvBridge
import numpy as np

class cvBridgeDemo():
    def __init__(self):
        self.node_name = "cv_bridge_demo"

        rospy.init_node(self.node_name)

        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)

        
        if rospy.has_param('~device') == True:
            device = rospy.get_param('~device')
            if not(device == 'kinect' or device == 'zed_stereo'):
                print("Sorry try. kinect/zed_stereo")
                sys.exit()

        else:
            print("Sorry device not found. Try kinect/zed_stereo")
            sys.exit()
        # Create the OpenCV display window for the RGB image
        #self.cv_window_name = self.node_name
        #cv.NamedWindow(self.cv_window_name, cv.CV_WINDOW_NORMAL)
        #cv.MoveWindow(self.cv_window_name, 25, 75)

        # Create the cv_bridge object
        self.bridge = CvBridge()

        # Subscribe to the camera image and depth topics and set
        # the appropriate callbacks
        
        if device == 'zed_stereo':
            self.image_sub = rospy.Subscriber("/zed/zed_node/left/image_rect_color", Image, self.image_callback)
        elif device == 'kinect':
            self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback)
        
        self.pub_indices = rospy.Publisher('/indices', Landmarks, queue_size=1)

    def image_callback(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
  

        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        imageCopy = np.array(frame, dtype=np.uint8)
        # Get dimensions of image
        image_width = imageCopy.shape[1]
        #print(image_width)
        image_height = imageCopy.shape[0]

        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_100)

        # Initialize the detector parameters using default values
        parameters =  cv2.aruco.DetectorParameters_create()

        # Detect the markers in the image
        markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(imageCopy, dictionary)
        
        idex_of_codes = []
        array_aux = []
        separators = []
        ids = []
        if np.all(markerIds is not None):
            
            
           
            for i in range(0, len(markerIds)):
                #rvec, tvec = cv.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.16, cameraMatrix, distCoeffs)
                #print("Rotation")
                #print(rvec)
                #print("Distancia :")
                #print(  math.sqrt( pow(tvec[0][0][0],2) + pow(tvec[0][0][1],2) + pow(tvec[0][0][2],2) )    )
                #print("Traslacion:")
                #print(tvec)
                imageCopy = cv2.aruco.drawDetectedMarkers(imageCopy, markerCorners, markerIds)
                #imageCopy = cv2.aruco.drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.01)  # Draw Axis
                min_x = markerCorners[i][0][0][0] 
                min_y = markerCorners[i][0][0][1] 

                max_x = markerCorners[i][0][2][0] 
                max_y = markerCorners[i][0][2][1]
                
                reducttion_x = 0#(max_x-min_x)/4
                reducttion_y = 0#(max_y-min_y)/4

                min_x += int(reducttion_x)
                max_x -= int(reducttion_x)

                min_y += int(reducttion_y)
                max_y -= int(reducttion_y)

                array_aux = np.arange( (max_y-min_y+1 )*(max_x-min_x+1 ) )

                j = 0
                for y in range(int(min_y),int(max_y)):
                    for x in range(int(min_x),int(max_x)):
                        array_aux[j] = image_width*y+(x)
                        j = j+1
                if i==0:
                    separators.append(j)
                else:
                    separators.append(separators[-1]+j)

                idex_of_codes=np.append (idex_of_codes, array_aux)
                ids.append(markerIds[i][0])

        landmarks_pub = Landmarks() 

        landmarks_pub.ids = ids
        landmarks_pub.separators = separators
        landmarks_pub.masks_index = idex_of_codes
        
        
        self.pub_indices.publish(landmarks_pub)
        cv2.imshow("out",imageCopy)



        # Process any keyboard commands
        self.keystroke = cv2.waitKey(5)
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'q':
                # The user has press the q key, so exit
                rospy.signal_shutdown("User hit q key to quit.")

    def cleanup(self):
        print "Shutting down vision node."
        cv2.destroyAllWindows()   

def main(args):       
    try:
        cvBridgeDemo()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)


