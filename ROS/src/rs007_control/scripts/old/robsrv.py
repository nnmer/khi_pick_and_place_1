from __future__ import print_function
import time
import zmq

tcpport = 10006

context = zmq.Context()
socket = context.socket(zmq.REP)
socket.bind("tcp://*:"+str(tcpport))

def killme(param):
     print("killing server")
     return "ok - kill"
     
def doj1(param):
    print("dojj1:"+param)
    return "ok - doj1"

def doj6(param):
    print("dojj6:"+param)   
    return "ok - doj6"

def mainloop():
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
