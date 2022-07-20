#!/usr/bin/python
from __future__ import print_function
import sys
import zmq
import json

socket = None
tcpport = 10006

cart_json_long = \
"{\r\n \"0\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.0,\r\n \"PathId\": 0,\r\n \"CartId\": 0\r\n },\r\n \"1\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.82717114686965942,\r\n \"PathId\": 4,\r\n \"CartId\": 1\r\n },\r\n \"2\": {\r\n \"Loaded\": false,\r\n \"Position\": 1.1695075035095215,\r\n \"PathId\": 1,\r\n \"CartId\": 2\r\n },\r\n \"3\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.47020155191421509,\r\n \"PathId\": 1,\r\n \"CartId\": 3\r\n },\r\n \"4\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.97200626134872437,\r\n \"PathId\": 1,\r\n \"CartId\": 4\r\n },\r\n \"5\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.66992568969726562,\r\n \"PathId\": 1,\r\n \"CartId\": 5\r\n },\r\n \"6\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.77113932371139526,\r\n \"PathId\": 1,\r\n \"CartId\": 6\r\n },\r\n \"7\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.87011498212814331,\r\n \"PathId\": 1,\r\n \"CartId\": 7\r\n },\r\n \"8\": {\r\n \"Loaded\": false,\r\n \"Position\": 1.0711431503295898,\r\n \"PathId\": 1,\r\n \"CartId\": 8\r\n },\r\n \"9\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.99256342649459839,\r\n \"PathId\": 5,\r\n \"CartId\": 9\r\n },\r\n \"10\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.57095009088516235,\r\n \"PathId\": 1,\r\n \"CartId\": 10\r\n },\r\n \"11\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.0,\r\n \"PathId\": 0,\r\n \"CartId\": 11\r\n }\r\n}"

cart_json_short = \
"{\r\n \"0\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.0,\r\n \"PathId\": 0,\r\n \"CartId\": 0\r\n },\r\n \"1\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.82717114686965942,\r\n \"PathId\": 4,\r\n \"CartId\": 1\r\n }\r\n}"

cart_json_short1 = \
"{\r\n \r\n \"10\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.57095009088516235,\r\n \"PathId\": 1,\r\n \"CartId\": 10\r\n },\"11\": {\r\n \"Loaded\": true,\r\n \"Position\": 0.67095009088516235,\r\n \"PathId\": 1,\r\n \"CartId\": 11\r\n },\"1\": {\r\n \"Loaded\": false,\r\n \"Position\": 0.82717114686965942,\r\n \"PathId\": 4,\r\n \"CartId\": 1\r\n }\r\n}"


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
    
    print("sending string "+msg)

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
        print("")
        print("send cart json - short version")
        print("   python robcli.py  cartjsons")
        print("")
        print("send cart json - long version")
        print("   python robcli.py  cartjsonl")
        print("")
        print(cart_json_short1)        
        jcart = json.loads(cart_json_short1)
        print(jcart)
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
    if msg=="cartjsons":
        msg = "cartjson" + "|" + cart_json_short1
        print("sending cart_json_short")
    if msg=="cartjsonl":
        print("sending cart_json_long")
        msg = "cartjson" + "|" + cart_json_long
    if nargs>2:
        msg = msg + "|" + sys.argv[2]
    sendmsg(msg)
    print("done")

main()