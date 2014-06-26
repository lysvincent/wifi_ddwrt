#!/usr/bin/python

import sys,string
#from scipy import spatial
from collections import defaultdict
import rospy
import roslaunch
from wifi_ddwrt.msg import *
import math
import numpy
import tf
import pickle
from geometry_msgs.msg import *
from nav_msgs.srv import *
from visualization_msgs.msg import *
from StringIO import StringIO

class PosEstimate:
  def __init__(self, database):
    #print "database=", database
    self.dic = pickle.load(open( database,"rb"))
    rospy.Subscriber('ddwrt/sitesurvey', SiteSurvey, self.survey_cb)
    #pub = rospy.Publisher('ddwrt/positions', Array)

    self.inv_fp = {}
    self.fp_keys = []
    self.l_rssi = []
    self.rssi_dict = {}
    #print dic.keys(), "=", type(dic.keys())
    #print "a=", type(a[0])
    #print "a=", type(list(a[0]))
    #print "b=", a[0]
    #print "b[0]=", a[0][0]

    #print "dic(b)=", dic[b], "is", type(dic[b])

  def invert_keys(self):
    inv_fp = defaultdict(list)
    #inv_fp = {v:k for k, v in self.dic.items()}
    keys_list = self.dic.keys()
    values_list = self.dic.values()
    for x in range(len(self.dic.items())):
      inv_fp[values_list[x]] = keys_list[x]
    #print "inverted_dic=", inv_fp.items()
    return inv_fp
    #print "rev_dic.keys=", fp_keys
    #print "first ap of the first key=", fp_keys[0][0]
    #print "the ssid=", fp_keys[0][0][0]
    #print "Rssi=", fp_keys[0][0][2]
    
  def get_aps_rssi_list(self,fp_keys):
    l_rssi=[]

    for fp_key in fp_keys:
      for ap in fp_key:
        l_rssi.append([row[])
        #print ap[2]
        l_rssi.append(ap[2])
    
    return l_rssi

  def match_rssi_to_pos(self,l_rssi):
    nt_conv_dict = defaultdict(list)
    l_values = self.dic.values()
    #nt_conv_dict = {v:k for v in l_rssi, k in l_values}
    for x in range(len(l_rssi)):
      nt_conv_dict[l_rssi[x]] = l_values[x]

    return nt_conv_dict

  def take(n, iterable):
    "Return first n items of the iterable as a list"
    a = list(islice(iterable, n))
    return a

  def convert(self,r_rssi_list):
    dic_rssi = numpy.asarray(self.rssi_dict.keys())
    fact_dic = {}
    r_rssi_list = numpy.asarray(r_rssi_list)
    for x in range(len(self.rssi_dict.keys())):
      dist = numpy.linalg.norm(r_rssi_list - dic_rssi[x])
      fact_dic[dist] = self.rssi_dict[dic_rssi[x]]
      
  
  def survey_cb(self, survey):

    r_networks = []

    for s_ap in survey.networks:
      if (s_ap.macaddr == rospy.get_param('macaddr_1') and rospy.get_param('essid_1'))  : r_networks.append((s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == rospy.get_param('macaddr_2') and rospy.get_param('essid_2')): r_networks.append((s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == rospy.get_param('macaddr_3') and rospy.get_param('essid_3')): r_networks.append((s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == rospy.get_param('macaddr_4') and rospy.get_param('essid_4')): r_networks.append((s_ap.macaddr,s_ap.rssi))
      elif (s_ap.macaddr == rospy.get_param('macaddr_5') and rospy.get_param('essid_5')): r_networks.append((s_ap.macaddr,s_ap.rssi))
      else : x = 0
    
    r_rssi_list = []
    r_networks = numpy.asarray(r_networks)

    for x in range(len(r_networks)):
      r_rssi_list.append(r_networks[x][1])

    pe.convert(r_rssi_list)
  
def approximate():
  rospy.init_node('approximate', anonymous=True)
  #stri = rospy.get_param('database')
  stri = "fingerprints.pkl"
  pe = PosEstimate(stri)
  pe.inv_fp = pe.invert_keys()
  pe.fp_keys = list(pe.inv_fp.keys()) 
  pe.l_rssi = pe.get_aps_rssi_list(pe.fp_keys)
  #print "len(l_rssi):", len(pe.l_rssi)
  #print "len(l_values):", len(pe.dic.values())
  pe.rssi_dict = pe.match_rssi_to_pos(pe.l_rssi)
  
     
if __name__ == '__main__':
  approximate()
  
