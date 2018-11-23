import telnetlib
from subprocess import call
import time

class BucklerTelnet:

  def __init__(self):
    self.telnet_conn = telnetlib.Telnet("localhost", 19021)
    # Clear initial output on telnet channel
    self.telnet_conn.read_until("JLinkExe\r\n") 

  def setSpeed(self, newSpeed):
    self.telnet_conn.write("setSpeed")
    self.telnet_conn.write(newSpeed)
  
  def setTurnSpeed(self, newSpeed):
    self.telnet_conn.write("turn spd")
    self.telnet_conn.write(newSpeed)

  def driveInfinite(self):
    self.telnet_conn.write("driveInf")
  
  def stop(self):
    self.telnet_conn.write("stop")

  def driveDist(self, dist):
    self.telnet_conn.write("driveDis")
    self.telnet_conn.write("%.2f" % dist)
  
  def turnRightInfinite(self):
    self.telnet_conn.write("rightInf")
  
  def turnLeftInfinite(self):
    self.telnet_conn.write("left Inf")
  
  def turnRightAngle(self, angle):
    self.telnet_conn.write("rightAng")
    self.telnet_conn.write("%.2f" % angle)
  
  def turnLeftAngle(self, angle):
    self.telnet_conn.write("leftAngl")
    self.telnet_conn.write("%.2f" % angle)
  
  def reverseInfinite(self):
    self.telnet_conn.write("reverInf")
  
  def reverseDist(self, dist):
    self.telnet_conn.write("reverDis")
    self.telnet_conn.write("%.2f" % dist)
  


buckler = BucklerTelnet()
buckler.reverseInfinite()
time.sleep(3)
buckler.stop()