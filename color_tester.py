# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
#from msvcrt import getch
#import curses

# initialize the camera and grab a reference to the raw camera capture
camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(640, 480))

#stdscr = curses.initscr()
#curses.cbreak()
#stdscr.keypad(1)

#stdscr.addstr(0,10,"Hit 'q' to quit")
#stdscr.refresh()

#key = ''

if len(sys.argv) == 1:
   hue_value = 1
else:
   for a in sys.argv[1:]:
      hue_value = (int(a))
estop = False

while not estop:
    while not estop:
        try:
            print ("hue value: ", hue_value)
            #hue_value = int(input("Hue value between 0 and 255: "))
            if (hue_value < 0) or (hue_value > 255):
                raise ValueError
        except ValueError:
            estop = True
            print("That isn't an integer between 0 and 255, try again")
            exit ()
        else:
            break

#for hue_value in range (1, 255):
#    print ("current hue: ", hue_value)
    lower_red = np.array([hue_value-10,100,100])
    upper_red = np.array([hue_value+10, 255, 255])
 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
      try:
        image = frame.array
 
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
 
        color_mask = cv2.inRange(hsv, lower_red, upper_red)
 
        result = cv2.bitwise_and(image, image, mask= color_mask)
 
        cv2.imshow("Camera Output", image)
        cv2.imshow("HSV", hsv)
        cv2.imshow("Color Mask", color_mask)
        cv2.imshow("Final Result", result)
 
        rawCapture.truncate(0)

        #key = stdscr.getch()
        k = cv2.waitKey(5) #& 0xFF
        #print ("key: ", k)
        if "q" == chr(k & 0xff):
            estop = True
            break
        if "n" == chr(k & 0xff):
            break
      except KeyboardInterrupt:
        estop = True
        break
    time.sleep (1)
    hue_value = hue_value + 1

#curses.endwin()
