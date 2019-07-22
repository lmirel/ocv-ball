
import serial

#!/usr/bin/python3
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s
    
    def readline(self):
        i = self.buf.find(b"\n")
        if i >= 0:
            r = self.buf[:i+1] #don't +1 if you don't need \n
            self.buf = self.buf[i+1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b"\n")
            if i >= 0:
                r = self.buf + data[:i+1] #don't +1 if you don't need \n
                self.buf[0:] = data[i+1:]
                return r
            else:
                self.buf.extend(data)

ser = serial.Serial ('/dev/ttyUSB0', 115200, timeout=1)
# open the serial port
if ser.isOpen ():
     print (ser.name + ' is open...')
"""
  //all values range from 0..1023 using ADC range
  //for other ranges, use mapping values to converge to these
  //    bytes number: 000011112222333344445555666677778888
  //                  lt-Xlt-Yrt-Xrt-Yrt-Plt-Pbtnsnullnull
  //e.g. full, right: 000010201020000005120512000000000000
  //e.g.  mid, ahead: 000010200512000005120380000000000000
  //e.g. full, ahead: 000010200512000010200380000000000000
"""

st = '{:04d}{:04d}{:04d}{:04d}{:04d}{:04d}{:04d}{:04d}{:04d}\r'.format (0, 1020, 512, 0, 512, 380, 0, 0, 0)
print (st)
#ser.write (st.encode ())

out = ''
dk = 0

srl = ReadLine (ser)

while True:
  try:
    cc = srl.readline()
    print ("dbg: " + cc.decode())
    dk = dk + 1
    if (dk > 8):
      #ser.write (st.encode ())
      dk = 0

  except KeyboardInterrupt:
    break

ser.close()
