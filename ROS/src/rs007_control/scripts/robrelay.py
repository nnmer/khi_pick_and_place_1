from __future__ import print_function
import time
import zmq
import rospy
import rosgraph
from rs007_control.msg import Rs007Joints6
from rs007_control.msg import Rs007Joints1

tcpport = 10006


def zmqinit():
  global socket
  context = zmq.Context()
  socket = context.socket(zmq.REP)
  socket.bind("tcp://*:"+str(tcpport))

J1_TOPIC_NAME = 'Rs007Joints1'
J6_TOPIC_NAME = 'Rs007Joints6'
NODE_NAME = 'joint_publisher'


def wait_for_connections(pub, topic):
    ros_master = rosgraph.Master('/rostopic')
    topic = rosgraph.names.script_resolve_name('rostopic', topic)
    num_subs = 0
    for sub in ros_master.getSystemState()[1]:
        print(num_subs)
        print(sub)
        if sub[0] == topic:
            num_subs+=1
    print("wait_for_connections num_subs:"+str(num_subs))

    for i in range(10):
        pub_num_connects = pub.get_num_connections()
        if pub_num_connects == num_subs:
            return
        time.sleep(0.1)
    print("pub_num_connects:"+str(pub_num_connects))
    # raise RuntimeError("failed to get publisher pub_num_connects:"+str(pub_num_connects))

def rosinit():
    global pub1,pub6, J1_TOPIC_NAME, J6_TOPIC_NAME
    rospy.init_node(NODE_NAME, anonymous=True)
    pub1 = rospy.Publisher(J1_TOPIC_NAME, Rs007Joints1, queue_size=10)
    wait_for_connections( pub1, J1_TOPIC_NAME)
    pub6 = rospy.Publisher(J6_TOPIC_NAME, Rs007Joints6, queue_size=10)
    wait_for_connections( pub6, J6_TOPIC_NAME)


def killme(param):
     print("killing server")
     return "ok - kill"
     
def doj1(param):
    global pub1
    print("dojj1:"+param)
    s2 = param.split(',')
    ln = len(s2)
    if ln!=2:
      return "Error - j1 - wrong number of values need 2 have :"+st(ln)
    idx = int(float(s2[0]))
    ang = float(s2[1])
    j1 = Rs007Joints1( idx,ang )
    pub1.publish(j1)       
    return "ok - doj1"

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
    pub6.publish(j6)       
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
        rv = "unknown command:"+cmd
        
        if cmd=="kill":
          rv = killme(parm)
        elif cmd=="j1":
          rv = doj1(parm)
        elif cmd=="j6":
          rv = doj6(parm)
          
        #  Do some 'work'
        time.sleep(0.001)

        #  Send reply back to client
        socket.send_string(rv)
        
        if cmd=="kill":
            exit()
            

mainloop()
