# !/usr/bin/env python

# PCA9685.py
# 2016-01-31
# Public Domain

import time

import pigpio

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

if __name__ == "__main__":

   import time

   import PCA9685
   import pigpio

   pi = pigpio.pi()

   if not pi.connected:
      exit(0)
   "steering params"
   st_min = 1100
   st_max = 1900
   st_stp = 50
   st_mid = st_min + (st_max - st_min) / 2
   "throttle params"
   th_max = 3500
   th_min = 2500
   th_brk = 0
   pwm = PCA9685.PWM(pi) # defaults to bus 1, address 0x40

   pwm.set_frequency(50) # suitable for servos



   """
   for dc in range (5, 11):
       print ("turn angle %d", dc) 
       pwm.set_duty_cycle(0, dc) # -1 for all channels
       time.sleep(.5)
   """
   "throttle control"
   "forward"
   print ("forward")
   pwm.set_pulse_width (4, 2500)
   pwm.set_pulse_width (5, 2500)
   pwm.set_pulse_width (6, 0)
   pwm.set_pulse_width (7, 0)
   time.sleep (1)
   "backward"
   print ("backward")
   pwm.set_pulse_width (4, 0)
   pwm.set_pulse_width (5, 0)
   pwm.set_pulse_width (6, 2500)
   pwm.set_pulse_width (7, 2500)
   time.sleep (1)
   "stop"
   print ("stop")
   pwm.set_pulse_width (4, 0)
   pwm.set_pulse_width (5, 0)
   pwm.set_pulse_width (6, 0)
   pwm.set_pulse_width (7, 0)
   time.sleep (1)
   
   """
   "steering control"
   for pw in range (st_min, st_max, st_stp):
     print ("turn angle %d", pw)
     pwm.set_pulse_width (0, pw) # -1 for all channels
     time.sleep(.1)
   
   pwm.set_pulse_width (0, st_mid);
   time.sleep (.5)
   """
   pwm.cancel()

   pi.stop()

