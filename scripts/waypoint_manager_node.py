#!/usr/bin/env python

import threading
import signal
import sys
import math

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from Tkinter import *
import tkFileDialog

C_EARTH = 6378137.0

def gps_convert_ned(gps_t_lon, gps_t_lat,
                    gps_r_lon, gps_r_lat):
  d_lon = gps_t_lon - gps_r_lon;
  d_lat = gps_t_lat - gps_r_lat;
  ned_x = (math.pi/180.)*d_lat * C_EARTH;
  ned_y = (math.pi/180.)*d_lon * C_EARTH * math.cos((math.pi/180.)*gps_t_lat);
  #s = 'ned_x: '+repr(ned_x) + ' ned_y: ' + repr(ned_y);
  #print s;
  return (ned_x, ned_y)


class RosThread(threading.Thread):
  def __init__(self):
    threading.Thread.__init__(self)
  def run(self):
    rospy.spin()

class WaypointManagerFrame(Frame):
  def __init__(self, parent):
    Frame.__init__(self, parent)   
         
    self.parent = parent
    self.curr_gps = NavSatFix()
    self.ref_gps = NavSatFix()
    self.gps_lock = threading.Lock()
    self.path_lock = threading.Lock()
    self.gps_msgs = [];
    self.path_pub = rospy.Publisher('waypoints', Path, queue_size=1, latch=True)
    self.initUI()

  def setRefGps(self, ref_gps):
    self.ref_gps = ref_gps
        
  def setGPS(self, gps):
    self.gps_lock.acquire()
    self.curr_gps = gps
    self.gps_lock.release()

  def getGPS(self):
    self.gps_lock.acquire()
    gps = self.curr_gps
    self.gps_lock.release()
    return gps

  def getGpsList(self):
    self.path_lock.acquire()
    gps_msgs = self.gps_msgs
    self.path_lock.release()
    return gps_msgs
  
  def publishWps(self):
    gps_msgs = self.getGpsList();
    path = Path()
    path.header.frame_id = 'world'
    for gm in gps_msgs:
      (x,y) = gps_convert_ned(
        gm.longitude,
        gm.latitude,
        self.ref_gps.longitude,
        self.ref_gps.latitude
        ); 
      y = -y
      z = gm.altitude
      pose = PoseStamped()
      pose.pose.position.x = x
      pose.pose.position.y = y
      pose.pose.position.z = z
      pose.header.frame_id = 'world'
      path.poses.append(pose)
    self.path_pub.publish(path)

  def insertCurrentGps(self):
    self.insertGps(self.getGPS())

  def insertGps(self, gps):
    self.path_lock.acquire()
    self.gps_msgs.append(gps)
    self.path_lock.release()
    self.gps_list.insert(END, str(gps.latitude) + " " + str(gps.longitude) + " " + str(gps.altitude))
    self.publishWps()

  def removeGps(self):
    if len(self.gps_msgs) > 0:
      self.path_lock.acquire()
      self.gps_msgs.pop()
      self.path_lock.release()
    self.gps_list.delete(END,END)
    self.publishWps()

  def clearGpsList(self):
    while len(self.gps_msgs) >0:
      self.removeGps()

  def saveWps(self):
    fn = tkFileDialog.asksaveasfilename(); 
    if fn:
      f = open(fn, 'w')
      gps_msgs = self.getGpsList()
      for gm in gps_msgs:
        f.write("{:.9f}".format(gm.latitude) + ", " + "{:.9f}".format(gm.longitude) + ", " + "{:.9f}".format(gm.altitude) + "\n")
      f.close()

  def openWps(self):
    fn = tkFileDialog.askopenfilename();
    if fn:
      self.clearGpsList()
      f = open(fn, 'r')
      for line in f:
        gps_msg = NavSatFix()
        parsed_list = line.split(",")
        gps_msg.latitude = float(parsed_list[0])
        gps_msg.longitude = float(parsed_list[1])
        gps_msg.altitude = float(parsed_list[2])
        self.insertGps(gps_msg)
        print line
      f.close()
    
        
  def initUI(self):      
    self.parent.title("Waypoint Manager")
    self.pack(fill=BOTH, expand=1)   

    self.gps_list = Listbox(self.parent)
    self.gps_list.pack(side=LEFT)    

    self.addButton = Button(self, text="Add",
        command=self.insertCurrentGps)
    self.addButton.pack(side=LEFT)
    self.removeButton = Button(self, text="Remove",
        command=self.removeGps)
    self.removeButton.pack(side=LEFT)
    self.saveButton = Button(self, text="Save", command=self.saveWps)
    self.saveButton.pack(side=LEFT)
    self.openButton = Button(self, text="Open", command=self.openWps)
    self.openButton.pack(side=LEFT)


def signal_handler(signal, frame):
  print('Killed')
  sys.exit(0)

root = Tk()
wpManager = WaypointManagerFrame(root)

def gpsCallback(data):
  wpManager.setGPS(data)
def gpsStartCallback(data):
  print "Got home GPS"
  wpManager.setRefGps(data)
  #gps_ref_sub.unsubscribe()

if __name__ == '__main__':
  signal.signal(signal.SIGINT, signal_handler)
  rospy.init_node('waypoint_manager')
  rospy.Subscriber("gps", NavSatFix, gpsCallback)
  gps_ref_sub = rospy.Subscriber("gps_ref", NavSatFix, gpsStartCallback)
 
  ros_thread = RosThread()
  ros_thread.daemon = True
  ros_thread.start()

  root.mainloop()
  root.destroy()

  ros_thread.join()
