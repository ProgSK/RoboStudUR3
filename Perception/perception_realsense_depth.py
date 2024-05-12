import pyrealsense2 as rs
import numpy as np
import cv2
from matplotlib import pyplot as plt
import imutils
from imutils import perspective
from imutils import contours
import math
import rospy
from std_msgs.msg import Float32

#Function find midpoint of two coordinates
def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

#Function finds distance between two points
def distance(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

#Function finds angle of a line between two points relative to x axis
def angle(pt1, pt2):
    angle_rad = math.atan2(pt1[1] - pt2[1],pt1[0] - pt2[0])
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360
    
    if angle_deg > 180:
        angle_deg = angle_deg - 180

    return angle_deg


##Variable initialisation 
height_to_centre = None #Variable storing height from camera to converyor 
box_height = None #Variable storing object height
centre_point = [300, 65] #Variable for Belt centre of RGB image 
centre_point_depth = [300, 40] #Variable for Belt centre of Depth image 
mm_per_pixel = 10/17 #Variable storing pixel size ratio from test measurement 


col_depth_offset = [0, 50]
#Add cm per pixel metric for xy Dimension\

#Enable camera streaming data
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)
pipe.start(cfg)


#While loop through each camera frame 
while True:

    #get colour and depth frame
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    #convert frame data to image
    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    #Crop image to just belt
    belt = color_image[15:160, 5:630]
    belt_depth = depth_image [110:200, 70:630]
    
    # 60 - 400 y
    # 10 - 600 x

    # x = 364, y = 165  depth 
    # 430, 110 = colour image
    # 

    
    #Concert belt depth to colour image
    belt_depth_colour = cv2.applyColorMap(cv2.convertScaleAbs(belt_depth,
                                     alpha = 0.5), cv2.COLORMAP_JET)

    #Changing Depth iamge to greyscale 
    img_gray_depth = cv2.cvtColor(belt_depth_colour, cv2.COLOR_BGR2GRAY)
    img_gray_depth = cv2.GaussianBlur(img_gray_depth, (7, 7), 0)
    _, threshold_depth = cv2.threshold(img_gray_depth, 185, 200, cv2.THRESH_BINARY)

    cv2.imshow("Image Depth grey", img_gray_depth)
    #Finding contours in depth image
    cnts_depth = cv2.findContours(threshold_depth, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts_depth = imutils.grab_contours(cnts_depth)


    depth_image_colour = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                     alpha = 0.5), cv2.COLORMAP_JET)
    


    if height_to_centre == None:
        height_to_centre = belt_depth[(centre_point_depth[1]), (centre_point_depth[0])]

    for c in cnts_depth:
        
        if cv2.contourArea(c) < 1000:
            continue

        orig_depth = belt_depth_colour.copy()
        box_depth = cv2.minAreaRect(c)
        box_depth = cv2.cv.BoxPoints(box_depth) if imutils.is_cv2() else cv2.boxPoints(box_depth)
        box_depth = np.array(box_depth, dtype="int")

        box_depth = perspective.order_points(box_depth)
        cv2.drawContours(orig_depth, [box_depth.astype("int")], -1, (0, 255, 0), 2)

        for (x, y) in box_depth:
            cv2.circle(orig_depth, (int(x), int(y)), 5, (0, 0, 255), -1)

        #Identify object vertices
        (tl_dep, tr_dep, br_dep, bl_dep) = box_depth
        (tltrX_dep, tltrY_dep) = midpoint(tl_dep, tr_dep)
        (blbrX_dep, blbrY_dep) = midpoint(bl_dep, br_dep)
        (tlblX_dep, tlblY_dep) = midpoint(tl_dep, bl_dep)
        (trbrX_dep, trbrY_dep) = midpoint(tr_dep, br_dep)

        #identify Object Centre
        (centx_dep, centy_dep) = midpoint(tl_dep, br_dep)

        if box_height == None and np.abs(centx_dep - centre_point_depth[0]) < 10:
            box_height = height_to_centre - belt_depth[int(centy_dep), int(centx_dep)]
            


        #Display circles/lines on object
        cv2.circle(orig_depth, (int(tltrX_dep), int(tltrY_dep)), 5, (255, 0, 0), -1)
        cv2.circle(orig_depth, (int(blbrX_dep), int(blbrY_dep)), 5, (255, 0, 0), -1)
        cv2.circle(orig_depth, (int(tlblX_dep), int(tlblY_dep)), 5, (255, 0, 0), -1)
        cv2.circle(orig_depth, (int(trbrX_dep), int(trbrY_dep)), 5, (255, 0, 0), -1)

        cv2.line(orig_depth, (int(tltrX_dep), int(tltrY_dep)), (int(blbrX_dep), int(blbrY_dep)),
            (255, 0, 255), 2)
        cv2.line(orig_depth, (int(tlblX_dep), int(tlblY_dep)), (int(trbrX_dep), int(trbrY_dep)),
            (255, 0, 255), 2)
        
        cv2.imshow("Image Depth", orig_depth)



        




    
    
    #Changing colour image to grey scale
    img_gray = cv2.cvtColor(belt, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    _, threshold = cv2.threshold(img_gray, 120, 300, cv2.THRESH_BINARY)

    #Finding Contours in colour image
    cnts = cv2.findContours(threshold, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)

    # (cnts, _) = contours.sort_contours(cnts)

    print(len(cnts))



    # height_to_centre = belt_depth[71, 310]

    #Display Feeds
    # cv2.imshow("belt", belt)
    # cv2.imshow("colour", color_image)
    cv2.imshow("belt depth", belt_depth )
    # cv2.imshow("shadow RGB", threshold)
    cv2.imshow("shadow depth", threshold_depth)
    

    for c in cnts:
        # if the contour is not sufficiently large, ignore it
        if cv2.contourArea(c) < 1000:
            continue
        

        orig = belt.copy()
        box = cv2.minAreaRect(c)
        box = cv2.cv.BoxPoints(box) if imutils.is_cv2() else cv2.boxPoints(box)
        box = np.array(box, dtype="int")

        box = perspective.order_points(box)
        cv2.drawContours(orig, [box.astype("int")], -1, (0, 255, 0), 2)
        
        for (x, y) in box:
            cv2.circle(orig, (int(x), int(y)), 5, (0, 0, 255), -1)
        
            
        (tl, tr, br, bl) = box
        (tltrX, tltrY) = midpoint(tl, tr)
        (blbrX, blbrY) = midpoint(bl, br)

        (tlblX, tlblY) = midpoint(tl, bl)
        (trbrX, trbrY) = midpoint(tr, br)

        print("tl",tl)
        print("tr",tr)
        print("br",br)
        print("bl",bl)

        (centx, centy) = midpoint(tl, br)
        
        print("Centres: x", int(centx),type(centx), "y", int(centy), type(centy))

        len1 = distance(tl, bl)
        len2 = distance(bl, br)
        

        

        if len1 > len2:
            theta = angle(tl, bl)
            Length = len1 * mm_per_pixel
            Width = len2 * mm_per_pixel


        else:
            theta = angle(bl, br)
            Width = len1 * mm_per_pixel
            Length = len2 * mm_per_pixel


        

        print("theta", theta, type(theta))
        print("Length", Length, type(Length))
        print("Width", Width, type(Width))

        #use the box centre as a radius from the frame centre like below

        # if box_height == None and np.abs(centx - centre_point[0]) < 10:
        #     box_height = height_to_centre - belt_depth[int(centy) + col_depth_offset[1], int(centx) + col_depth_offset[0]]
        #     print("LOOP SUCCESS")

        # print("depth", belt_depth[150, 360])

        # print("CENT X",centx )
        # print("Centrepoint",centre_point[1])

        # if box_height == None and centx > centre_point[0]:
        #     box_height = height_to_centre - depth_image[int(centx), int(centy)]

        # if centx > centre_point[0]:
        #     box_height = height_to_centre - belt_depth[int(centx), int(centy)]

        print("height to centre", height_to_centre)
        print("Box height", box_height, type(box_height))
        

        # theta1 = angle(bl, br)
        # theta2 = angle(tl, bl)
        # print("theta1", theta1)
        # print("theta2", theta2)


        cv2.circle(orig, (int(tltrX), int(tltrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(blbrX), int(blbrY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(tlblX), int(tlblY)), 5, (255, 0, 0), -1)
        cv2.circle(orig, (int(trbrX), int(trbrY)), 5, (255, 0, 0), -1)

        cv2.line(orig, (int(tltrX), int(tltrY)), (int(blbrX), int(blbrY)),
            (255, 0, 255), 2)
        cv2.line(orig, (int(tlblX), int(tlblY)), (int(trbrX), int(trbrY)),
            (255, 0, 255), 2)
        
        cv2.imshow("Image", orig)
    




    # cv2.imshow('rgb', color_image)
    # cv2.imshow('depth', depth_cm)

    if cv2.waitKey(1) == ord('q'):
        break

pipe.stop()