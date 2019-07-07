from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import time
import numpy as np
#import gpiozero

class PWM:

   """
   This class provides an interface to the I2C PCA9685 PWM chip.

   The chip provides 16 PWM channels.

   All channels use the same frequency which may be set in the
   range 24 to 1526 Hz.

   If used to drive servos the frequency should normally be set
   in the range 50 to 60 Hz.

   The duty cycle for each channel may be independently set
   between 0 and 100%.

   It is also possible to specify the desired pulse width in
   microseconds rather than the duty cycle.  This may be more
   convenient when the chip is used to drive servos.

   The chip has 12 bit resolution, i.e. there are 4096 steps
   between off and full on.
   """

   _MODE1         = 0x00
   _MODE2         = 0x01
   _SUBADR1       = 0x02
   _SUBADR2       = 0x03
   _SUBADR3       = 0x04
   _PRESCALE      = 0xFE
   _LED0_ON_L     = 0x06
   _LED0_ON_H     = 0x07
   _LED0_OFF_L    = 0x08
   _LED0_OFF_H    = 0x09
   _ALL_LED_ON_L  = 0xFA
   _ALL_LED_ON_H  = 0xFB
   _ALL_LED_OFF_L = 0xFC
   _ALL_LED_OFF_H = 0xFD

   _RESTART = 1<<7
   _AI      = 1<<5
   _SLEEP   = 1<<4
   _ALLCALL = 1<<0

   _OCH    = 1<<3
   _OUTDRV = 1<<2

   def __init__(self, pi, bus=1, address=0x40):

      self.pi = pi
      self.bus = bus
      self.address = address

      self.h = pi.i2c_open(bus, address)

      self._write_reg(self._MODE1, self._AI | self._ALLCALL)
      self._write_reg(self._MODE2, self._OCH | self._OUTDRV)

      time.sleep(0.0005)

      mode = self._read_reg(self._MODE1)
      self._write_reg(self._MODE1, mode & ~self._SLEEP)

      time.sleep(0.0005)

      self.set_duty_cycle(-1, 0)
      self.set_frequency(200)

   def get_frequency(self):

      "Returns the PWM frequency."

      return self._frequency

   def set_frequency(self, frequency):

      "Sets the PWM frequency."

      prescale = int(round(25000000.0 / (4096.0 * frequency)) - 1)

      if prescale < 3:
         prescale = 3
      elif prescale > 255:
         prescale = 255

      mode = self._read_reg(self._MODE1);
      self._write_reg(self._MODE1, (mode & ~self._SLEEP) | self._SLEEP)
      self._write_reg(self._PRESCALE, prescale)
      self._write_reg(self._MODE1, mode)

      time.sleep(0.0005)

      self._write_reg(self._MODE1, mode | self._RESTART)

      self._frequency = (25000000.0 / 4096.0) / (prescale + 1)
      self._pulse_width = (1000000.0 / self._frequency)

   def set_duty_cycle(self, channel, percent):

      "Sets the duty cycle for a channel.  Use -1 for all channels."

      steps = int(round(percent * (4096.0 / 100.0)))

      if steps < 0:
         on = 0
         off = 4096
      elif steps > 4095:
         on = 4096
         off = 0
      else:
         on = 0
         off = steps

      if (channel >= 0) and (channel <= 15):
         self.pi.i2c_write_i2c_block_data(self.h, self._LED0_ON_L+4*channel,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8])

      else:
         self.pi.i2c_write_i2c_block_data(self.h, self._ALL_LED_ON_L,
            [on & 0xFF, on >> 8, off & 0xFF, off >> 8])

   def set_pulse_width(self, channel, width):

      "Sets the pulse width for a channel.  Use -1 for all channels."

      self.set_duty_cycle(channel, (float(width) / self._pulse_width) * 100.0)

   def cancel(self):

      "Switches all PWM channels off and releases resources."

      self.set_duty_cycle(-1, 0)
      self.pi.i2c_close(self.h)

   def _write_reg(self, reg, byte):
      self.pi.i2c_write_byte_data(self.h, reg, byte)

   def _read_reg(self, reg):
      return self.pi.i2c_read_byte_data(self.h, reg)


camera = PiCamera()
image_width = 640
image_height = 480
camera.resolution = (image_width, image_height)
camera.framerate = 32
camera.rotation = 180
rawCapture = PiRGBArray(camera, size=(image_width, image_height))
center_image_x = image_width / 2
center_image_y = image_height / 2
min_radius = 5
max_radius = 100
minimum_area = 250
maximum_area = 50000
 
import PCA9685
import pigpio
pi = pigpio.pi()
if not pi.connected:
  exit(0)
pwm = PCA9685.PWM(pi) # defaults to bus 1, address 0x40
pwm.set_frequency(50) # suitable for servos
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
lower_color = np.array([HUE_VAL-10,100,100])
upper_color = np.array([HUE_VAL+10, 255, 255])
for frame in camera.capture_continuous (rawCapture, format="bgr", use_video_port=True):
  try:
    image = frame.array
    #+snip
    # resize the frame, blur it, and convert it to the HSV
    # color space
    #frame = imutils.resize(frame, width=600)
    #frame = imutils.rotate(frame, angle=180)
    blurred = cv2.GaussianBlur(image, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # construct a mask for the color "green", then perform
    # a series of dilations and erosions to remove any small
    # blobs left in the mask
    colmask = cv2.inRange (hsv, lower_color, upper_color)
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
       object_x = c_x
       object_y = c_y
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
        #cv2.circle(image, (int(object_x), int(object_y)), int(20),
        #    (0, 255, 255), 2)
        #show the frame to our screen
        #cv2.circle (image, (int(object_x), int(object_y)), int(20),
        #    (0, 255, 255), 2)
        #cv2.imshow("Frame", image)
        #
        #result
        #result = cv2.bitwise_and (image, image, mask=colmask)
        #cv2.circle (result, (int(c_x), int(c_y)), int(object_r), (0, 255, 255), 2)
        #cv2.imshow ("Result", result)
        key = cv2.waitKey (1) & 0xFF
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

    rawCapture.truncate(0)

  except KeyboardInterrupt:
    break

pwm.cancel()

pi.stop()
