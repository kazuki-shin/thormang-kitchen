#!/usr/bin/env python
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge
bridge = CvBridge()

def reject_borders(image_):
        out_image = image_.copy()
        h, w = image_.shape[:2]
        for row in range(h):
            if out_image[row, 0] == 255:
                cv2.floodFill(out_image, None, (0, row), 0)
            if out_image[row, w - 1] == 255:
                cv2.floodFill(out_image, None, (w - 1, row), 0)
        for col in range(w):
            if out_image[0, col] == 255:
                cv2.floodFill(out_image, None, (col, 0), 0)
            if out_image[h - 1, col] == 255:
                cv2.floodFill(out_image, None, (col, h - 1), 0)
        return out_image

def pixel2world(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) 

    #adaptive thresholding
    bw = cv2.adaptiveThreshold(gray, 255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 7, -1) #cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    bw = cv2.bitwise_not(bw)

    #remove components on image border
    bw2 = reject_borders(bw)

    #fill holes
    im_floodfill = bw2.copy()
    h, w = bw2.shape[:2]
    mask = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(im_floodfill, mask, (0,0), 255)
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    bw3 = bw2 | im_floodfill_inv

    #remove small noise
    kernel1 = np.ones((10,10),np.uint8)
    bw4 = cv2.morphologyEx(bw3, cv2.MORPH_OPEN, kernel1)

    #get connected components
    nb_components, output, stats, centroids = cv2.connectedComponentsWithStats(bw4, connectivity=8)
    sizes = stats[1:, -1]; nb_components = nb_components - 1
    width = stats[1:,  cv2.CC_STAT_WIDTH]
    height = stats[1:,  cv2.CC_STAT_HEIGHT]
    
    #find cup
    min_size = 5000  
    centroid = [0,0]

    cupBW = np.zeros((output.shape))
    for i in range(0, nb_components-1):
        if sizes[i] >= min_size:
            cupBW[output == i + 1] = 255
            centroid = centroids[i + 1]
            w = width[i]
            h = height[i]


    u = centroid[0]
    v = centroid[1]

    #linear fit model
    x_w = 720.6+0.009141*u-0.8645*v
    y_w =  355.6-0.541*u-0.009774*v

    return cupBW,[u,v,x_w,y_w,w,h],bw2

#set up publishers
rospy.init_node('viz', anonymous=True)
cupPub = rospy.Publisher('viz/cup', Image, queue_size=10)
coordsPub = rospy.Publisher('viz/coords', Pose , queue_size=10)

points = []

def callback(data):
    global points

    if (data.header.stamp)<rospy.Time.from_sec(1636841419):
        points = []

    cv_image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    cupBW,coords,bw = pixel2world(cv_image)


    coord = Pose()
    coord.position.x = coords[2]
    coord.position.y = coords[3]

    #rectagnle coordinates
    w = coords[4]+10
    h = coords[5]+10
    top_left = (max(0,int(coords[0]-w/2)),max(0,int(coords[1]-h/2)))
    bottom_right = (int(coords[0]+w/2),int(coords[1]+h/2))

    if (coords[0] != 0) and (coords[1] != 0):
        #draw centroid and rectangle
        points.append((coords[0],coords[1]))
        img = cv2.circle(cv_image, (int(coords[0]),int(coords[1])), radius=3, color=(0, 0, 255), thickness=-1)
        img  = cv2.rectangle(img,top_left,bottom_right,(0,255,0),3)
        font = cv2.FONT_HERSHEY_SIMPLEX
        img = cv2.putText(img,"x_w: "+str(int(coords[2]))+" y_w: "+str(int(coords[3])),(10,500), font, 1, (0, 0, 255), 2, cv2.LINE_AA)
        
        #publish centroid
        coordsPub.publish(coord)
    img = cv2.circle(cv_image, (int(coords[0]),int(coords[1])), radius=3, color=(0, 0, 255), thickness=-1)


    #show feed
    cv2.imshow("image",img)
    #cv2.imshow("bw",cupBW)
    #cv2.imshow("bw1",bw)
    cv2.waitKey(10)
    

    cup = bridge.cv2_to_imgmsg(cupBW, encoding="passthrough")
    cupPub.publish(cup)



    
def listener():
    rospy.Subscriber("/robotis/sensor/camera/image_raw", Image, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
