#!/usr/bin/env python

import random
import rospy
import rosgraph
import time
from rs007_control.msg import Rs007Joints1


TOPIC_NAME = 'Rs007Joints1'
NODE_NAME = 'joint1_publisher'


def post_joint():
    pub = rospy.Publisher(TOPIC_NAME, Rs007Joints1, queue_size=10)
    rospy.init_node(NODE_NAME, anonymous=True)

    idx = random.randint(0, 5)
    ang = random.randint(0, 360)
    j1 = Rs007Joints1(idx,ang)

    wait_for_connections(pub, TOPIC_NAME)
    pub.publish(j1)
    print("published joint idx:"+str(idx)+" ang:"+str(ang)+" to "+TOPIC_NAME)

    time.sleep(0.1)


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


if __name__ == '__main__':
    try:
        post_joint()
    except rospy.ROSInterruptException:
        pass
