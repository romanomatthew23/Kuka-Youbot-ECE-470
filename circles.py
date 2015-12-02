#!/usr/bin/python

'''
This example illustrates how to use cv2.HoughCircles() function.
Usage: ./houghcircles.py [<image_name>]
image argument defaults to ../data/board.jpg
'''

import cv2.cv as cv
import cv2
import numpy as np
import sys

def is_appropriate_size(cnt,scale):
    return cv2.contourArea(cnt)*scale > .15 and cv2.contourArea(cnt)*scale < .35

def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs( np.dot(d1, d2) / np.sqrt( np.dot(d1, d1)*np.dot(d2, d2) ) )

def find_squares(img2,scale):
    img = cv2.GaussianBlur(img2, (5, 5), 0)
    squares = []
    for gray in cv2.split(img):
        for thrs in xrange(0, 255, 26):
            if thrs == 0:
                bin = cv2.Canny(gray, 0, 50, apertureSize=5)
                bin = cv2.dilate(bin, None)
            else:
                retval, bin = cv2.threshold(gray, thrs, 255, cv2.THRESH_BINARY)
            contours, hierarchy = cv2.findContours(bin, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                cnt_len = cv2.arcLength(cnt, True)
                cnt = cv2.approxPolyDP(cnt, 0.02*cnt_len, True)
                if len(cnt) == 4 and is_appropriate_size(cnt,scale) and cv2.isContourConvex(cnt):
                    cnt = cnt.reshape(-1, 2)
                    max_cos = np.max([angle_cos( cnt[i], cnt[(i+1) % 4], cnt[(i+2) % 4] ) for i in xrange(4)])
                    if max_cos < 0.1:
                        squares.append(cnt)
    #print squares
    return squares




#print __doc__
try:
    fn = sys.argv[1]
except:
    fn = "/home/youbot/libfreenect-inst2/libfreenect/libfreenect/build/bin/dataloc/output.png"


src = cv2.imread(fn, 1)
img = cv2.cvtColor(src, cv2.COLOR_BGR2GRAY)
img = cv2.medianBlur(img, 5)
cimg = src.copy() # numpy function

circles = cv2.HoughCircles(img, cv.CV_HOUGH_GRADIENT, 1, 10, np.array([]), 100, 30, 1, 30)
a, b, c = circles.shape

#verify location of square

if b != 4:
    sys.exit();

# find center
X_C = 0
Y_C = 0
for i in range(b):
    X_C += circles[0][i][0]
    Y_C += circles[0][i][1]
X_C /= 4.0;
Y_C /= 4.0;
for i in range(b):
    circles[0][i][0] -= X_C;
    circles[0][i][1] -= Y_C;

#rotation

ROT = 0
for i in range(b):
    ROT += (np.arctan2(circles[0][i][1],circles[0][i][0])) % np.pi/2
ROT /= 4.0
#print(ROT)
ROT -= np.pi/4
ROT *= -1
#print(ROT)
for i in range(b):
    circles[0][i][0],circles[0][i][1] = (np.cos(ROT)*circles[0][i][0] - np.sin(ROT)*circles[0][i][1],np.sin(ROT)*circles[0][i][0] + np.cos(ROT)*circles[0][i][1]);

#scaling
X_SC = 0
Y_SC = 0
for i in range(b):
    X_SC += np.abs(circles[0][i][0])
    Y_SC += np.abs(circles[0][i][1])
X_SC /= 4.0
Y_SC /= 4.0
X_SC = 1/X_SC
Y_SC = 1/Y_SC
for i in range(b):
    circles[0][i][0] *= X_SC
    circles[0][i][1] *= Y_SC


squares = find_squares(cv2.imread(fn),X_SC*Y_SC)
if not len(squares):
    sys.exit();

blk = squares[0];
X_BC = 0
Y_BC = 0
for i in range(4):
    X_BC += blk[i][0]
    Y_BC += blk[i][1]
X_BC /= 4.0;
Y_BC /= 4.0;

#print (X_BC,Y_BC)
(X_BC_OLD, Y_BC_OLD) = (X_BC,Y_BC)
X_BC -= X_C;
Y_BC -= Y_C;
X_BC,Y_BC = (np.cos(ROT)*X_BC - np.sin(ROT)*Y_BC,np.sin(ROT)*X_BC + np.cos(ROT)*Y_BC);
X_BC *= X_SC
Y_BC *= Y_SC
X_BC = (X_BC + 1.0)/2.0;
Y_BC = (Y_BC + 1.0)/2.0;
print int(1000000*Y_BC)
print int(1000000*X_BC)
#print ""
#for i in range(b):
    #print circles[0][i][0],circles[0][i][1]
cv2.drawContours(cimg, [blk], -1, (0, 255, 0), 3 )
cv2.circle(cimg, (int(X_C), int(Y_C)), 2, (0, 255, 0), 3, cv2.CV_AA) # draw
cv2.circle(cimg, (int(X_BC_OLD), int(Y_BC_OLD)), 2, (255, 0, 0), 3, cv2.CV_AA) # draw 

#cv2.imshow("source", src)
#cv2.imshow("detected circles", cimg)
cv2.waitKey(0)
