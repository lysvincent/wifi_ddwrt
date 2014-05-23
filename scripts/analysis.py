#!/usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of Willow Garage, Inc. nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
MIN_RSSI = 10
MAX_RSSI = 100

import rospy
from wifi_ddwrt.msg import *

import math
import tf
from geometry_msgs.msg import *
from nav_msgs.srv import *
from visualization_msgs.msg import *
from StringIO import StringIO

import Image
import ImageDraw

aps = { "08:60:6e:cc:79:04" : ("ap1", (255, 0, 0), 36), #red  cob-developer
        "84:c9:b2:6a:80:e8" : ("ap2", (0, 255, 0), 36), #lime cob3-3-extern
        "84:c9:b2:6a:81:30" : ("ap3", (0, 0, 255), 40), #blue desire-extern
        "00:0b:0e:c9:38:c1" : ("ap4", (255, 255, 0), 40), #yellow eduroam
        "00:0b:0e:c9:75:03" : ("ap5", (255, 0, 255), 44), #hot pink IZS-Campus
        "00:0b:0e:c9:75:05" : ("ap6", (0, 255, 255), 44), #aqua blue IZS-Mobil
        "00:0b:0e:c9:75:07" : ("ap7", (121, 71, 0), 44), #brown voip
        "00:0b:0e:c9:75:09" : ("ap8", (255, 150, 0), 44), #orange IZS-Gast
        "00:0b:0e:c9:75:0d" : ("ap9", (19, 90, 29), 44), #forest green privateIAO
        "00:0b:0e:c9:75:0f" : ("ap10", (122, 19, 176), 44),#purple privateIAOMAC
        "00:0b:0e:c9:03:81" : ("ap11", (246, 179, 197), 44),#light pink or salmon eduroam
        "00:0b:0e:c9:03:83" : ("ap12", (5, 29, 139), 44),#navy blue IZS-Campus
        "00:0b:0e:c9:03:85" : ("ap13", (128, 128, 0), 44),#olive IZS-Mobil
	"00:0b:0e:c9:03:87" : ("ap14", (255, 69, 0), 44),#orange red voip       
	"00:0b:0e:c9:03:8d" : ("ap15", (0, 255, 127), 44),#spring green privateIAO
	"00:0b:0e:c9:03:8f" : ("ap16", (65, 105, 225), 44),#royal blue privateIAOMAC
	"00:0b:0e:c8:fd:01" : ("ap17", (138, 43, 226), 48),#blue violet eduroam
	"00:0b:0e:c8:fd:03" : ("ap18", (123, 104, 238), 48),#medium slate blue IZS-Campus
	"00:0b:0e:c8:fd:05" : ("ap19", (139, 69, 19), 48),#saddle brown IZS-Mobil
	"00:0b:0e:c8:fd:07" : ("ap20", (173, 255, 47), 48),#green yellow voip
	"00:0b:0e:c8:fd:09" : ("ap21", (0, 128, 128), 48),#Teal IZS-Gast
	"00:0b:0e:c8:fd:0d" : ("ap22", (0, 128, 0), 48),#green privateIAO
	"00:0b:0e:c8:fd:0f" : ("ap23", (85, 107, 47), 48),#dark olive green privateIAOMAC
	"00:0b:0e:c9:38:c3" : ("ap24", (210, 105, 30), 40), #chocolate IZS-Campus 
        "24:77:03:57:ea:00" : ("ap25", (210, 105, 30), 36), #chocolate cob3-3-extern
	"fc:f8:ae:e4:a7:7d" : ("ap26", (85, 107, 47), 36), #dark olive green cob3-3-extern
	"f0:7b:cb:4e:58:28" : ("ap27", (0, 128, 0),   36)  #green cob3-3-extern
												}


def world_to_map(wx, wy, resolution):
  mx = int(wx - resp.map.info.origin[0] / resolution)
  my = int(wy - resp.map.info.origin[1] / resolution)
  return (mx, my) 

class WifiAnalysis:
  def __init__(self, listener):
    self.aps = aps
    self.listener = listener
    self.positions = []
    self.fingerprints = {}
    self.get_map()
    rospy.Subscriber('ddwrt/sitesurvey', SiteSurvey, self.survey_cb)
    rospy.Subscriber('ddwrt/accesspoint', AccessPoint, self.ap_cb)
    self.vis_pub = rospy.Publisher('visualization_marker', Marker)
    self.marker_count = 0

  def get_map(self):
    rospy.wait_for_service('static_map')

    try:
      map_service = rospy.ServiceProxy('static_map', GetMap)
      print "Requesting the static map"
      resp = map_service()
      
      size = (resp.map.info.width, resp.map.info.height)
      self.map_res = resp.map.info.resolution

      # Convert map to greyscale image
      new_data=[]
      for point in resp.map.data:
      	  new_data.append(point)

      remap = {-1: 120, 0: 255, 100: 0}
      dummy = lambda x: remap.get(new_data[x], 120)      
      data_list = []
    
      for x in range(len(new_data)): 
          remap = {-1: 120, 0: 255, 100: 0}      
	  data_list.append(dummy(x))
 
     

      self.im = Image.new('L', size)
      self.im.putdata(data_list)
           
      # Resize image and convert to RGB
      #self.im = self.im.resize((size[0]/2, size[1]/2), Image.BICUBIC)
      self.im = self.im.convert("RGB")								     #image is converted into RGB
      

    except rospy.ServiceException, e:
      print "The service call to get the map failed"	


  def save_map(self):
    last_ap = None

    for pos in self.positions:									     #positions are a list of macaddr,trans and ap.signal()
      map_coords = world_to_map(pos[1][0], pos[1][1], self.map_res)				     #pos[1][0]=trans[0]
      signal_quality = (MAX_RSSI - -1.0 * pos[2]) / (MAX_RSSI - MIN_RSSI)
      cell_radius = int(signal_quality * 0.1 / self.map_res)
      
      draw = ImageDraw.Draw(self.im)								     #labelling the image
      top_left = (map_coords[0] - cell_radius, map_coords[1] - cell_radius)
      bottom_right = (map_coords[0] + cell_radius, map_coords[1] + cell_radius)
      
      draw.rectangle((top_left, bottom_right), fill=self.aps[pos[0]][1])
      if last_ap != pos[0]:
        size = 0.5 / self.map_res
        draw.ellipse(((map_coords[0] - size, map_coords[1] - size), 				     #draw ellipse
            (map_coords[0] + size, map_coords[1] + size)), fill=self.aps[pos[0]][1])
        if last_ap != None:
          msg = "%s:%d-%s:%d" % (aps[last_ap][0], aps[last_ap][2], aps[pos[0]][0], aps[pos[0]][2])
        else:
          msg = "%s:%d " % (aps[pos[0]][0], aps[pos[0]][2])
        width, height = draw.textsize(msg)
        draw.text((map_coords[0]-(width)/2, map_coords[1]-(height)/2), msg, fill="white")
        last_ap = pos[0] 
    
    self.im.save("static_map.png", "PNG")


  def ap_cb(self, ap):
    #we need to get the pose of the robot at the time the survey came in
    self.listener.waitForTransform('/map', '/base_link', ap.header.stamp, rospy.Duration(2.0))
    try:
      (trans, rot) = self.listener.lookupTransform('/map', '/base_link', ap.header.stamp)
    except (tf.LookupException, tf.ConnectivityException):
      print "Got an exception that should never happen"
      return

    if not self.aps.has_key(str(ap.macaddr)):							      #if the current ap doesn't exist on the list, then add it into the list
      self.aps[str(ap.macaddr)] = ("ap" + str(len(self.aps) + 1), (20, 30, 0))

    self.positions.append((str(ap.macaddr), trans, ap.signal))

    signal_quality = (MAX_RSSI - -1.0 * ap.signal) / (MAX_RSSI - MIN_RSSI)
    radius = 0.1
    step_size = radius + 0.05


    x = trans[0]
    y = trans[1]
    z_scale = 0.25

    for i in range(int(signal_quality / .18)):
      #we'll also publish a visualization marker
      marker = Marker()
      marker.header.frame_id = "/map"
      marker.header.stamp = ap.header.stamp
      marker.ns = "wifi_analysis"
      marker.id = self.marker_count
      self.marker_count += 1
      marker.type = Marker.CUBE
      marker.action = Marker.ADD
      marker.pose.position.x = x
      marker.pose.position.y = y
      marker.pose.position.z = z_scale / 2.0
      marker.pose.orientation.x  = rot[0]
      marker.pose.orientation.y = rot[1]
      marker.pose.orientation.z = rot[2]
      marker.pose.orientation.w = rot[3]

      marker.scale.x = radius 
      marker.scale.y = radius
      marker.scale.z = z_scale
      marker.color.a = 1.0
     
      marker.color.r = aps[str(ap.macaddr)][1][0]
      marker.color.g = aps[str(ap.macaddr)][1][1]
      marker.color.b = aps[str(ap.macaddr)][1][2]
      self.vis_pub.publish(marker)

      angles = tf.transformations.euler_from_quaternion(rot)
      x += step_size * math.cos(angles[2])
      y += step_size * math.sin(angles[2])
      z_scale += (i + 1) * 0.05

    print trans

  def survey_cb(self, survey):
    #we need to get the pose of the robot at the time the survey came in
    try:
      (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException):
      print "Got an exception that should never happen"
      return

    r_networks = []    

    for s_ap in survey.networks:
      if (s_ap.macaddr == "08:60:6e:cc:79:04" and s_ap.essid == "cob-developer"): r_networks.append((s_ap.essid,s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == "84:c9:b2:6a:80:e8" and s_ap.essid == "cob3-3-extern"): r_networks.append((s_ap.essid,s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == "84:c9:b2:6a:81:30" and s_ap.essid == "desire-extern"): r_networks.append((s_ap.essid,s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == "00:0b:0e:c9:03:83" and s_ap.essid == "IZS-Campus")   : r_networks.append((s_ap.essid,s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == "00:0b:0e:c9:38:c1" and s_ap.essid == "eduroam")   : r_networks.append((s_ap.essid,s_ap.macaddr,s_ap.rssi))
      else : x = 0

    last_trans=[0,0]
    diff_x=abs(trans[0]-last_trans[0])
    diff_y=abs(trans[1]-last_trans[1])
    if diff_x > 1 or diff_y > 1:
    	self.fingerprints[trans]=r_networks
	last_trans=trans
	
    if self.fingerprints.keys():
	    print self.fingerprints.keys()[0]

def analysis():
  rospy.init_node('wifi_analysis', anonymous=True)

  listener = tf.TransformListener()
  wa = WifiAnalysis(listener) 
  rospy.spin()
  wa.save_map()
  print "saved"

if __name__ == '__main__':
  analysis()
