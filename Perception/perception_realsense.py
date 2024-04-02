import pyrealsense2 as rs
import numpy as np
import cv2
from matplotlib import pyplot as plt
import imutils
from imutils import perspective
from imutils import contours
import math

def midpoint(ptA, ptB):
    return ((ptA[0] + ptB[0]) * 0.5, (ptA[1] + ptB[1]) * 0.5)

def distance(pt1, pt2):
    return math.sqrt((pt1[0] - pt2[0])**2 + (pt1[1] - pt2[1])**2)

def angle(pt1, pt2):
    angle_rad = math.atan2(pt1[1] - pt2[1],pt1[0] - pt2[0])
    angle_deg = math.degrees(angle_rad)

    if angle_deg < 0:
        angle_deg += 360
    
    if angle_deg > 180:
        angle_deg = angle_deg - 180

    return angle_deg

height_to_centre = None
box_height = None
centre_point = [12, 58]

pipe = rs.pipeline()
cfg = rs.config()


cfg.enable_stream(rs.stream.color, 640,480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640,480, rs.format.z16, 30)

pipe.start(cfg)



while True:
    frame = pipe.wait_for_frames()
    depth_frame = frame.get_depth_frame()
    color_frame = frame.get_color_frame()

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    belt = color_image[15:155, 5:630]
    belt_depth = depth_image[15:155, 5:630]
    # 60 - 400 y
    # 10 - 600 x
    cv2.imshow("belt", belt)
    
    depth_cm = cv2.applyColorMap(cv2.convertScaleAbs(depth_image,
                                     alpha = 0.5), cv2.COLORMAP_JET)

    img_gray = cv2.cvtColor(belt, cv2.COLOR_BGR2GRAY)
    img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)
    
    _, threshold = cv2.threshold(img_gray, 70, 300, cv2.THRESH_BINARY)
    
    cv2.imshow("shadow", threshold)

    cnts = cv2.findContours(threshold, cv2.RETR_EXTERNAL,
    cv2.CHAIN_APPROX_SIMPLE)

    cnts = imutils.grab_contours(cnts)

    # (cnts, _) = contours.sort_contours(cnts)

    print(len(cnts))

    if height_to_centre == None:
        height_to_centre = belt_depth[12, 58]

    height_to_centre = belt_depth[71, 310]


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
        
        print("centres: x", centx, "y", centy)

        len1 = distance(tl, bl)
        len2 = distance(bl, br)
        
        

        if len1 > len2:
            theta = angle(tl, bl)
            print("first one")

        else:
            theta = angle(bl, br)
            print("second one")

        

        print("theta", theta)

        # if box_height == None and centx > centre_point[0]:
        #     box_height = height_to_centre - depth_image[int(centx), int(centy)]

        # if centx > centre_point[0]:
        #     box_height = height_to_centre - belt_depth[int(centx), int(centy)]

        print("heigh to centre", height_to_centre)
        print("Box height", box_height)
        

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