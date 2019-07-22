#!/usr/bin/env python3

#prereqs:
"""
pip3 install Adafruit_PCA9685
"""

import cv2
import time
import numpy as np
#import gpiozero

#image processing lib
def hisEqulColor(img):
    ycrcb = cv2.cvtColor (img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split (ycrcb)
    #print (len(channels))
    cv2.equalizeHist (channels[0], channels[0])
    #clahe = cv2.createCLAHE (clipLimit = 2.0, tileGridSize = (8, 8))
    #channels[0] = clahe.apply (channels [0])
    cv2.merge (channels, ycrcb)
    cv2.cvtColor (ycrcb, cv2.COLOR_YCR_CB2BGR, img)
    return img

#main code logic
if __name__ == "__main__":
    camera = cv2.VideoCapture(0)
    image_width = 640
    image_height = 480
    center_image_x = image_width / 2
    center_image_y = image_height / 2
    min_radius = 5
    max_radius = 100
    minimum_area = 250
    maximum_area = 50000
    
    import PCA9685
    #import pigpio
    #pig = pigpio.pi()
    #if not pig.connected:
    #  exit(0)
    pwm = PCA9685.PCA9685 (address=0x40, busnum=1)
    pwm.set_pwm_freq (60) # suitable for servos
    #robot = gpiozero.Robot(left=(22,27), right=(17,18))
    
    #movement speed
    speed_min = 8000
    forward_speed = 9000
    speed_max = 10000
    #steering
    steer_min = 1100
    go_straight = 1500
    steer_max = 1900
    turn_speed = 200
    
    #orange: 10
    #red: 175
    HUE_VAL = 175
    
    #car control
    def car_control (turn_angle, move_speed):
        #steering
        pwm.set_pulse_width (0, go_straight + turn_angle)
        #left motor
        pwm.set_pulse_width (4, move_speed)
        #right motor
        pwm.set_pulse_width (5, move_speed)
        #forward motion
        pwm.set_pulse_width (6, 0)
        pwm.set_pulse_width (7, 0)
    
    #integer mapping
    def int_map (val, in_min, in_max, out_min, out_max):
       return int((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)
    
    #camera processing
    #this is RED!
    lower_col1 = np.array ([0,  50,  50])
    upper_col1 = np.array ([10, 255, 255])
    #
    lower_col2 = np.array ([170, 50,  50])
    upper_col2 = np.array ([180, 255, 255])
    
    #lower_color = np.array([HUE_VAL-10,100,100])
    #upper_color = np.array([HUE_VAL+10, 255, 255])
    while True:
      ret, frame = camera.read()
      try:
        image = frame
        #frame processing logic
        hisimg = hisEqulColor (image.copy())
        #image = hisimg
        #
        blurred = cv2.GaussianBlur (hisimg, (11, 11), 0)
        hsv = cv2.cvtColor (blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        # lower mask (0-10)
        mask0 = cv2.inRange (hsv, lower_col1, upper_col1)
        # upper mask (170-180)
        mask1 = cv2.inRange (hsv, lower_col2, upper_col2)
        # join my masks
        colmask = mask0 + mask1
        #+snip
        # resize the frame, blur it, and convert it to the HSV
        # color space
        #frame = imutils.resize(frame, width=600)
        #frame = imutils.rotate(frame, angle=180)
        #blurred = cv2.GaussianBlur(image, (11, 11), 0)
        #hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        #colmask = cv2.inRange (hsv, lower_color, upper_color)
        colmask = cv2.erode (colmask, None, iterations=2)
        colmask = cv2.dilate (colmask, None, iterations=2)
    
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        #cnts = cv2.findContours (mask.copy(), cv2.RETR_EXTERNAL,
        #    cv2.CHAIN_APPROX_SIMPLE)
        #-snip
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #
        #color_mask = cv2.inRange(hsv, lower_color, upper_color)
        #find contours
        image2, cnts, hierarchy = cv2.findContours (colmask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        #our object
        object_r = 0
        object_x = 0
        object_y = 0
        #find object, pick the largest
        if len(cnts) > 0:
           cnts_max = max (cnts, key = cv2.contourArea)
           ((c_x, c_y), c_r) = cv2.minEnclosingCircle (cnts_max)
           object_x = int (c_x)
           object_y = int (c_y)
           object_r = int (c_r) #int (3.1415926535 * (c_r * c_r))
        #look for largest area we see
        #for contour in cnts:
        #    x, y, width, height = cv2.boundingRect(contour)
        #    found_area = width * height
        #    center_x = x + (width / 2)
        #    center_y = y + (height / 2)
        #    #print ("contour: ", cnt_k)
        #    #print ("center x: ", center_x)
        #    #print ("center y: ", center_y)
        #    #store largest area
        #    if object_area < found_area:
        #        object_area = found_area
        #        object_x = center_x
        #        object_y = center_y
        #
        #print ("object area: ", object_area)
        if object_r > 0:
            ball_location = [object_r, object_x, object_y]
            #>debug
            #cv2.circle(image, (object_x, object_y), object_r,
            #    (0, 255, 255), 2)
            #show the frame to our screen
            #cv2.circle (image, (int(object_x), int(object_y)), int(20),
            #    (0, 255, 255), 2)
            #cv2.imshow("Frame", image)
            #
            #result
            result = cv2.bitwise_and (image, image, mask=colmask)
            cv2.circle (result, (int(c_x), int(c_y)), int(object_r), (0, 255, 255), 2)
            cv2.imshow ("Result", result)
            key = cv2.waitKey (1) & 0xFF
            #<debug
        else:
            ball_location = None
     
        if ball_location:
            if (ball_location[0] > min_radius) and (ball_location[0] < max_radius):
                st_dir = int_map (ball_location[1], 0, image_width, steer_min, steer_max)
                mv_spd = int_map (object_r, min_radius, max_radius, speed_max, speed_min)
                #if ball_location[1] > (center_image_x + (image_width/4)):
                #    #robot.right(turn_speed)
                #    print("Turning right")
                #    car_control (turn_speed, forward_speed)
                #elif ball_location[1] < (center_image_x - (image_width/4)):
                #    #robot.left(turn_speed)
                #    print("Turning left")
                #    car_control (-turn_speed, forward_speed)
                #else:
                #    #robot.forward(forward_speed)
                #    print("Forward")
                #    car_control (st_dir, forward_speed)
                print ("steering direction: ", st_dir, "object radius: ", object_r, "move speed: ", mv_spd)
                car_control (st_dir - go_straight, mv_spd)
    
            elif (ball_location[0] < min_radius):
                #robot.left(turn_speed)
                print("Target isn't large enough, searching")
                #stop car
                car_control (0, 0)
            else:
                #robot.stop()
                print("Target large enough, stopping")
                #stop car
                car_control (0, 0)
        else:
            #robot.left(turn_speed)
            print("Target not found, searching")
            #stop car
            car_control (0, 0)
    
        #rawCapture.truncate(0)
    
      except KeyboardInterrupt:
        break
    
    pwm.cancel()

