#!/usr/bin/python

import sys,string
import scipy
from scipy import spatial
from collections import defaultdict
import rospy
import roslaunch
from wifi_ddwrt.msg import *
import math
import tf
import pickle
from geometry_msgs.msg import *
from nav_msgs.srv import *
from visualization_msgs.msg import *
from StringIO import StringIO

dic = pickle.load(open("fingerprints.pkl","rb"))

#print dic.keys(), "=", type(dic.keys())
a = dic.keys()
b = a[0]
#print "a=", type(a[0])
#print "a=", type(list(a[0]))
#print "b=", a[0]
#print "b[0]=", a[0][0]

#print "dic(b)=", dic[b], "is", type(dic[b])

inv_fp = defaultdict(list)
inv_fp = {v:k for k, v in dic.items()}
fp_keys=list(inv_fp.keys())
#print "rev_dic.keys=", fp_keys
print "first ap of the first key=", fp_keys[0][0]
#print "the ssid=", fp_keys[0][0][0]
print "Rssi=", fp_keys[0][0][2]

l_rssi=[]

for fp_key in fp_keys:
    print fp_key
    for ap in fp_key:
        print ap[2]
        l_rssi.append(ap[2])


