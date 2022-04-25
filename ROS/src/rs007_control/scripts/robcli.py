# from __future__ import print_function
import sys
import zmq

socket = None
tcpport = 10006

def setupsocket():
    global socket
    context = zmq.Context()

    #  Socket to talk to server
    # print("Conection to sever")
    socket = context.socket(zmq.REQ)
    socket.connect("tcp://localhost:"+str(tcpport))
    # print("socket setup")



def sendmsg(msg):
    global socket
    
    # print("sending string "+msg)

    socket.send_string(msg)

    		#  Get the reply.
    reply = socket.recv()
    print("Received reply %s [ %s ]" % (msg, reply))


def help(errmsg=""):
    if (errmsg!=""):
        print(errmsg)
        print(sys.argv[0]+" : send messages to ROS system to control robot arm")
        print("There are two forms of message")
        print("Move one joint by specifying the joint number and the degree value it should be set to")
        print("   python robcli.py  j1 1,2")
        print("")
        print("Move six joint by specifying the value all 6 should be set to")
        print("   python robcli.py  j6 45,0,0,45,0,90")
    exit()

def main():
    global socket
    nargs = len(sys.argv)
    # print('robcli starting with:', nargs, 'parameters.')
    # print('   parameter List:', str(sys.argv))   
    print('')
    if nargs<2:
        help("Error: Need a command")
    setupsocket()
    msg = sys.argv[1]
    if nargs>2:
        msg = msg + "|" + sys.argv[2]
    sendmsg(msg)
    print("done")

main()