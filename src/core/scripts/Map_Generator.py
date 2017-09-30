#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from smba_core.msg import *


def talker():
    pub = rospy.Publisher('ground_truth', smba_object_list, queue_size=10)
    rospy.init_node('Map_Generator', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    send_id = 0
    while not rospy.is_shutdown():
        # initialize header for the message.
        msg_out = smba_object_list()
        msg_out.head.msg_id = send_id
        msg_out.head.generation_time = rospy.get_time()
        msg_out.head.last_modification = rospy.get_time()
        # generate and fill object with values
        test_object = smba_object()
        test_object.pos.x = 10
        test_object.pos.y = 0
        test_object.heading = 0
        test_object.width = 2
        test_object.length = 3
        test_object.speed = 100
        test_object.object_type = 0
        msg_out.data.append(test_object)
        # generate and fill object with values
        test_object = smba_object()
        test_object.pos.x = 30
        test_object.pos.y = 1
        test_object.heading = 0
        test_object.width = 2
        test_object.length = 3
        test_object.speed = 100
        test_object.object_type = 0
        # add object to msg_out
        msg_out.data.append(test_object)
        # generate and fill object with values
        test_object = smba_object()
        test_object.pos.x = -10
        test_object.pos.y = 0
        test_object.heading = 0
        test_object.width = 2
        test_object.length = 3
        test_object.speed = 100
        test_object.object_type = 0
        # add object to msg_out
        msg_out.data.append(test_object)

        # send object
        pub.publish(msg_out)
        send_id += 1
        rospy.loginfo(sys.argv)
        rate.sleep()
   
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
