# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import sys
#from msvcrt import getch
#import curses

def hisEqulColor(img):
    ycrcb = cv2.cvtColor (img, cv2.COLOR_BGR2YCR_CB)
    channels = cv2.split (ycrcb)
    #print (len(channels))
    #cv2.equalizeHist (channels[0], channels[0])
    clahe = cv2.createCLAHE (clipLimit = 2.0, tileGridSize = (64, 64))
    channels[0] = clahe.apply (channels [0])
    cv2.merge (channels, ycrcb)
    cv2.cvtColor (ycrcb, cv2.COLOR_YCR_CB2BGR, img)
    return img

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
   hue_value1 = 1
   hue_value2 = 1
else:
   for a in sys.argv[1:]:
      hue_value1 = (int(a))
      hue_value2 = (int(a))
estop = False

while not estop:
    while not estop:
        try:
            print ("hue value: ", hue_value1)
            #hue_value = int(input("Hue value between 0 and 255: "))
            if (hue_value1 < 0) or (hue_value1 > 255):
                raise ValueError
        except ValueError:
            estop = True
            print("That isn't an integer between 0 and 255, try again")
            exit ()
        else:
            break

#for hue_value in range (1, 255):
#    print ("current hue: ", hue_value)
    lower_col1 = np.array([hue_value1,100,100])
    upper_col1 = np.array([hue_value1 + 10, 255, 255])
    #
    lower_col2 = np.array([hue_value2,100,100])
    upper_col2 = np.array([hue_value2 + 10, 255, 255])
 
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
      try:
        image = frame.array
        #snip
        #create a CLAHE object (Arguments are optional).
        #clahe = cv2.createCLAHE (clipLimit=2.0, tileGridSize=(8,8))
        #im1 = np.uint16(image)
        #cl1 = clahe.apply(im1)
        ##
        #m_img = cv2.medianBlur (image,5)
        #rt, th1 = cv2.threshold (m_img, 180, 255, cv2.THRESH_BINARY)
        #th2 = cv2.inpaint (m_img, th1, 9, cv2.INPAINT_TELEA)
        #rv, th2 = cv2.threshold (image, 12, 255, cv2.THRESH_BINARY)
        #blurred = cv2.GaussianBlur (th2, (11, 11), 0)
        hisimg = hisEqulColor (image.copy())
        image = hisimg
        #
        blurred = cv2.GaussianBlur (image, (11, 11), 0)
        hsv = cv2.cvtColor (blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        # lower mask (0-10)
        lower_red = np.array ([0,  50,  50])
        upper_red = np.array ([10, 255, 255])
        mask0 = cv2.inRange (hsv, lower_red, upper_red)
        # upper mask (170-180)
        lower_red = np.array ([170, 50,  50])
        upper_red = np.array ([180, 255, 255])
        mask1 = cv2.inRange (hsv, lower_red, upper_red)
        # join my masks
        cmask = mask0 + mask1
        #
        #cmask = cv2.inRange (hsv, lower_col, upper_col)
        cmask = cv2.erode (cmask, None, iterations=2)
        cmask = cv2.dilate (cmask, None, iterations=2)
        #snip
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        #color_mask = cv2.inRange(hsv, lower_col, upper_col)
        result = cv2.bitwise_and (image, image, mask=cmask)
 
        cv2.imshow("Camera Output", image)
        #cv2.imshow("Clahe", cl1)
        #cv2.imshow("Threshold", th2)
        cv2.imshow("EQC", hisimg)
        cv2.imshow("HSV", hsv)
        cv2.imshow("Color Mask", cmask)
        cv2.imshow("Final Result", result)
 
        rawCapture.truncate(0)

        #key = stdscr.getch()
        k = cv2.waitKey(5) #& 0xFF
        #print ("key: ", k)
        if "q" == chr(k & 0xff):
            estop = True
            break
        if "p" == chr(k & 0xff):
            hue_value = hue_value - 1
            break
        if "n" == chr(k & 0xff):
            hue_value = hue_value + 1
            break
      except KeyboardInterrupt:
        estop = True
        break
    time.sleep (1)

#curses.endwin()
