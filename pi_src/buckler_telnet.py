
import telnetlib
from subprocess import call
import time

class BucklerTelnet:

  def __init__(self):
    self.telnet_conn = telnetlib.Telnet("localhost", 19021)
    # Clear initial output on telnet channel
    while(len(self.telnet_conn.read_eager())):
        self.telnet_conn.read_eager()
    self.stop_set = False

  def setSpeed(self, newSpeed):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"setSpeed")
    self.telnet_conn.write(bytes("{}".format(newSpeed), "utf-8"))
  
  def setTurnSpeed(self, newSpeed):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"turn spd")
    self.telnet_conn.write(newSpeed)

  def driveInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"driveInf")
  
  def stop(self):
    self.telnet_conn.write(b"stop")

  def driveDist(self, dist):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"driveDis")
    self.telnet_conn.write(b"%.2f" % dist)
    self.blockUntilDone()
  
  def turnRightInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"rightInf")
  
  def turnLeftInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"left Inf")
  
  def turnRightAngle(self, angle):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"rightAng")
    self.telnet_conn.write(b"%.2f" % angle)
    time.sleep(1)
    self.blockUntilDone()
  
  def turnLeftAngle(self, angle):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"leftAngl")
    self.telnet_conn.write(b"%.2f" % angle)
    self.blockUntilDone()
  
  def reverseInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"reverInf")
  
  def reverseDist(self, dist):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write(b"reverDis")
    self.telnet_conn.write(b"%.2f" % dist)
    self.blockUntilDone()

  def resetGrabber(self):
    self.telnet_conn.write(b"rst_grab")
    time.sleep(3)
  
  def rotateGrabber(self):
    self.telnet_conn.write(b"grabserv")
    time.sleep(3)
  
  def liftCup(self):
    self.telnet_conn.write(b"liftserv")
    time.sleep(3)
  
  def resetLift(self):
    self.telnet_conn.write(b"rst_lift")
    time.sleep(3)
  
  def blockUntilDone(self):
    msg = self.telnet_conn.read_eager()
    while not len(msg):
     msg = self.telnet_conn.read_eager() 
  

#buckler = BucklerTelnet()
#time.sleep(3)
#buckler.resetLift()
#buckler.rotateGrabber()
#buckler.liftCup()
#buckler.resetGrabber()
#buckler.driveDist(.1)
#buckler.driveDist(.1)
#buckler.turnRightAngle(20)
#buckler.driveDist(.1)
#buckler.driveDist(.1)
#buckler.driveDist(.1)
#buckler.driveDist(.1)
#buckler.driveDist(.1)
#buckler.driveDist(.1)
