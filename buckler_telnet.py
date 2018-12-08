import telnetlib
from subprocess import call
import time

class BucklerTelnet:

  def __init__(self):
    self.telnet_conn = telnetlib.Telnet("localhost", 19021)
    # Clear initial output on telnet channel
    self.telnet_conn.read_until("JLinkExe\r\n") 
    self.stop_set = False

  def setSpeed(self, newSpeed):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("setSpeed")
    self.telnet_conn.write(newSpeed)
  
  def setTurnSpeed(self, newSpeed):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("turn spd")
    self.telnet_conn.write(newSpeed)

  def driveInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("driveInf")
  
  def stop(self):
    self.telnet_conn.write("stop")

  def driveDist(self, dist):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("driveDis")
    self.telnet_conn.write("%.2f" % dist)
    self.blockUntilDone()
  
  def turnRightInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("rightInf")
  
  def turnLeftInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("left Inf")
  
  def turnRightAngle(self, angle):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("rightAng")
    self.telnet_conn.write("%.2f" % angle)
    self.blockUntilDone()
  
  def turnLeftAngle(self, angle):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("leftAngl")
    self.telnet_conn.write("%.2f" % angle)
    self.blockUntilDone()
  
  def reverseInfinite(self):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("reverInf")
  
  def reverseDist(self, dist):
    msg = self.telnet_conn.read_eager()
    if msg == 'STOP':
      self.stop_set = not self.stop_set
    if self.stop_set:
      self.stop()
      return 
    self.telnet_conn.write("reverDis")
    self.telnet_conn.write("%.2f" % dist)
    self.blockUntilDone()

  def resetGrabber(self):
    self.telnet_conn.write("rst_grab")
    time.sleep(3)
  
  def rotateGrabber(self):
    self.telnet_conn.write("grabserv")
    time.sleep(3)
  
  def liftCup(self):
    self.telnet_conn.write("liftserv")
    time.sleep(3)
  
  def resetLift(self):
    self.telnet_conn.write("rst_lift")
    time.sleep(3)
  
  def blockUntilDone(self):
    msg = self.telnet_conn.read_eager()
    while msg != "DONE":
     msg = self.telnet_conn.read_eager() 
  

