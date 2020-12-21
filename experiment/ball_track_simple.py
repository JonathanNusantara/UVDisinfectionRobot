import cv2
#import cv2.cv as cv
import numpy as np

def average_distance(distance_list, length): # Average every five entry
    total = 0
    for i in distance_list:
        total += i
    return total / length

# Constants
kernel_morph = np.ones((3,3),np.uint8)
focal = 220
width_ball = 3.35

# Take input from webcam
cap = cv2.VideoCapture(-1) # 640x480

# Reduce the size of video to 320x240 so rpi can process faster
cap.set(3,320)
cap.set(4,240)

# variables and list
ball_x = [] # x axis coordinate of ball center
ball_rad = [] # radius of ball center
dist_x = 0
dist_l = 0
dist_r = 0
dist_rad = 0
dist_calc = 0

while(1):

    _, frame = cap.read()

    # Detect based on color
    greenLower = (19, 91, 0) #values for tennis ball
    greenUpper = (34, 255, 78)
    #tracking = cv2.inRange(hsv, greenLower, greenUpper)
    
    # blurred = cv2.medianBlur(frame, 13) # Much slower
    blurred = cv2.GaussianBlur(frame, (7, 7), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, kernel_morph, iterations=2)
    closing = cv2.dilate(mask, kernel_morph, iterations=5)
    closing = cv2.GaussianBlur(closing,(11,11),0)

    # Some morpholigical filtering
    #dilation = cv2.dilate(tracking,kernel,iterations = 1)
    #closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)
    #closing = cv2.GaussianBlur(closing,(5,5),0)

    # Detect circles using HoughCircles
    # param is threshold of size of circle
    # If param2 is low, more sensitive to small circles and false positive
    circles = cv2.HoughCircles(closing,cv2.HOUGH_GRADIENT,2,120,param1=100,param2=40,minRadius=2,maxRadius=0) 
    # circles = np.uint16(np.around(circles))
    

    #Draw Circles
    if circles is not None:
            for i in circles[0,:]:
                cv2.circle(frame,(int(round(i[0])),int(round(i[1]))),int(round(i[2])),(0,255,0),5) # x is 320 y is 240, draw circle
                ball_x.append(round(i[0])) # Append x center coordinate to list
                ball_rad.append(round(i[2])) # Append radius to list
                
                # Average every 5 entries
                if len(ball_x) == 5: 
                    dist_x = average_distance(ball_x, 5) # x coordinate of ball center
                    ball_x = []
                    
                    # Calculated distance of ball from camera
                    dist_rad = average_distance(ball_rad, 5) 
                    dist_calc = width_ball * focal / dist_rad
                    #print(dist_calc)
                    ball_rad = []
                    
                    # For output to control algorithm
                    dist_r = 320 - (dist_x + dist_rad)
                    dist_l = dist_x - dist_rad
                    #print(dist_l)
                    
                    # Focal distance calculation
                    # focal = (dist_ave * 25.4 / 3.35) # in pixels and cm


    cv2.imshow('closing',closing)
    cv2.imshow('tracking',frame)

    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()
