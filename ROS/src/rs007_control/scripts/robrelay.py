#!/usr/bin/env python
from __future__ import print_function
import time
import zmq
import json
import rospy
import rosgraph
from rs007_control.msg import Rs007Joints6
from rs007_control.msg import Rs007Joints1
from rs007_control.msg import MagneMotionSled
from rs007_control.msg import MagneMotionTray
from rs007_control.msg import MagneMotionTray1

zmqport = rospy.get_param("ZMQ_PORT",10006)


def zmqinit():
  global socket
  context = zmq.Context()
  socket = context.socket(zmq.REP)
  socket.bind("tcp://*:"+str(zmqport))

J1_TOPIC_NAME = 'Rs007Joints1'
J6_TOPIC_NAME = 'Rs007Joints6'
SLED_TOPIC_NAME = 'Rs007Sleds'
TRAY_TOPIC_NAME = 'Rs007Tray'
TRAY1_TOPIC_NAME = 'Rs007Tray1'
NODE_NAME = 'joint_publisher'


def wait_for_connections(pub, topic):
    ros_master = rosgraph.Master('/rostopic')
    topic = rosgraph.names.script_resolve_name('rostopic', topic)
    num_subs = 0
    for sub in ros_master.getSystemState()[1]:
        # print(num_subs)
        # print(sub)
        if sub[0] == topic:
            num_subs+=1
    print("wait_for_connections num_subs:"+str(num_subs))

    for i in range(10):
        pub_num_connects = pub.get_num_connections()
        if pub_num_connects == num_subs:
            return
        time.sleep(0.1)
    print("      pub_num_connects:"+str(pub_num_connects))
    # raise RuntimeError("failed to get publisher pub_num_connects:"+str(pub_num_connects))

def rosinit():
    global pub1con,pub6con, sledcon,traycon,tray1con
    global J1_TOPIC_NAME, J6_TOPIC_NAME, SLED_TOPIC_NAME, TRAY_TOPIC_NAME, TRAY1_TOPIC_NAME
    rospy.init_node(NODE_NAME, anonymous=True)
    pub1con = rospy.Publisher(J1_TOPIC_NAME, Rs007Joints1, queue_size=10)
    wait_for_connections( pub1con, J1_TOPIC_NAME)
    pub6con = rospy.Publisher(J6_TOPIC_NAME, Rs007Joints6, queue_size=10)
    wait_for_connections( pub6con, J6_TOPIC_NAME)
    sledcon = rospy.Publisher(SLED_TOPIC_NAME, MagneMotionSled, queue_size=10)
    wait_for_connections( sledcon, SLED_TOPIC_NAME)
    traycon = rospy.Publisher(TRAY_TOPIC_NAME, MagneMotionTray, queue_size=10)
    wait_for_connections( traycon, TRAY_TOPIC_NAME)
    tray1con = rospy.Publisher(TRAY1_TOPIC_NAME, MagneMotionTray1, queue_size=10)
    wait_for_connections( traycon, TRAY1_TOPIC_NAME)    

def killme(param):
     print("killing server")
     return "ok - kill"
     
def doj1(param):
    global pub1
    print("dojj1:"+param)
    s2 = param.split(',')
    ln = len(s2)
    if ln!=2:
      return "Error - j1 - wrong number of values need 2 have :"+str(ln)
    idx = int(float(s2[0]))
    ang = float(s2[1])
    j1 = Rs007Joints1( idx,ang )
    pub1con.publish(j1)       
    return "ok - doj1"
    
def str_to_bool(s):
    rv = True
    s = s.lower()
    if (s=="false" or s=="0" or s=="f"):
      rv = false
    return rv
    
def docart1(param):
    global sledcon
    print("docart1:"+param)
    s2 = param.split(',')
    ln = len(s2)
    if ln!=4:
      return "Error - docart1 - wrong number of values need 4 have :"+str(ln)
    loaded = str_to_bool(s2[0])
    position = float(s2[1])
    pathid = int(float(s2[2]))    
    sledid = int(float(s2[3]))    
    mmmsg = MagneMotionSled( loaded,position,pathid,sledid )
    sledcon.publish(mmmsg)       
    return "ok - docart1"
    
def docartjson(param):
    print("docartjson:...")
    global sledcon
    cartdict = json.loads(param)
    # print(cartdict)
    ncarts = 0
    for key,cart in cartdict.items():
        print("  key:"+str(key)+" cart:"+str(cart))
        loaded = cart["Loaded"]
        position = cart["Position"]
        pathid = cart["PathId"]   
        sledid = cart["CartId"]    
        mmmsg = MagneMotionSled( loaded,position,pathid,sledid ) 
        sledcon.publish(mmmsg)      
        ncarts += 1
    return "ok - docartjson processed "+str(ncarts)+" carts"    
 
def dotrayjson(param):
    print("dotrayjson:"+param)
    global tray1con
    traylist = json.loads(param.lower())
    print(traylist)
    ntrays = 0
    itray = 0
    for traystat in traylist:
        row = int(itray / 4)    
        col = int(itray % 4)    
        # print(str(itray)+": "+str(row)+" "+str(col)+": "+str(traystat))
        loaded = int(traystat)
        mmmsg = MagneMotionTray1( row, col, loaded )
        tray1con.publish(mmmsg)       
        itray += 1
        ntrays += 1
    return "ok -  dotrayjson processed "+str(ntrays)+" trays"  
 
def dotray1(param):
    global tray1con
    print("dotray1:"+param)
    s2 = param.split(',')
    ln = len(s2)
    if ln!=3:
      return "Error - dotray1 - wrong number of values need 3 have :"+str(ln)
    row = int(float(s2[0]))    
    col = int(float(s2[1]))    
    loaded = int(float(s2[2])) 
    mmmsg = MagneMotionTray1( row, col, loaded )
    tray1con.publish(mmmsg)       
    return "ok - dotray1"    
    
def doj6(param):
    global pub6
    print("dojj6:"+param)
    s6 = param.split(',')
    ln = len(s6)
    if ln!=6:
      return "Error - j6 - wrong number of values need 6 have :"+st(ln)
    ang0 = float(s6[0])
    ang1 = float(s6[1])
    ang2 = float(s6[2])
    ang3 = float(s6[3])
    ang4 = float(s6[4])
    ang5 = float(s6[5])
    j6 = Rs007Joints6( [ang0,ang1,ang2,ang3,ang4,ang5] )
    pub6con.publish(j6)       
    return "ok - doj6"

def mainloop():
    global socket
    zmqinit()
    rosinit()
    while True:
        #  Wait for next request from client
        print("Waiting for request")
        message = socket.recv()
        
        message = message.decode("utf-8")
            
        sar = message.split("|")
        cmd = sar[0]
        parm = ""
        if len(sar)>1:
            parm = sar[1]
            
        rv = "not set"
        
        try:
          if cmd=="kill":
            rv = killme(parm)
          elif cmd=="j1":
            rv = doj1(parm)
          elif cmd=="j6":
            rv = doj6(parm)
          elif cmd=="cart1" or cmd=="sled1":
            rv = docart1(parm)
          elif cmd=="cartjson" or cmd=="sledjson":
            rv = docartjson(parm)   
          elif cmd=="trayjson":
            rv = dotrayjson(parm)              
          elif cmd=="tray1":
            rv = dotray1(parm)                 
          else:
            rv = "unknown command:"+cmd
        except Exception as ex:
            rv = "Exception occured:"+repr(ex)
          
        #  Do some 'work'
        time.sleep(0.001)

        #  Send reply back to client
        socket.send_string(rv)
        
        if cmd=="kill":
            exit()
            

mainloop()
