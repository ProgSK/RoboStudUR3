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



img = cv2.imread("box_angled_background.jpg")

img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_gray = cv2.GaussianBlur(img_gray, (7, 7), 0)

edged = cv2.Canny(img_gray, 50, 100)
edged = cv2.dilate(edged, None, iterations=1)
edged = cv2.erode(edged, None, iterations=1)

_, threshold = cv2.threshold(img_gray, 200, 300, cv2.THRESH_BINARY)



cv2.imshow("thresh", threshold)
cv2.waitKey(0)

cnts = cv2.findContours(threshold, cv2.RETR_EXTERNAL,
	cv2.CHAIN_APPROX_SIMPLE)
cnts = imutils.grab_contours(cnts)

(cnts, _) = contours.sort_contours(cnts)

print(len(cnts))

for c in cnts:
	# if the contour is not sufficiently large, ignore it
	if cv2.contourArea(c) < 1000:
		continue
	

	orig = img.copy()
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
	cv2.waitKey(0)








# _, threshold = cv2.threshold(img_gray, 100, 200, cv2.THRESH_BINARY)

# contours = cv2.findContours(img_gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# for cnt in contours:         
#     print(cnt)

# cv2.imshow("Image", img_rgb)
