#!/usr/bin/env python


## Simple talker demo that listens to std_msgs/Strings published 
## to the 'chatter' topic

import rospy #Import rospy if you are writing a ROS Node
from std_msgs.msg import String #so we can reuse std_msgs/String message type (A simple string container) for publishing

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def listener():

    # <2> In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.


    #This declares that your ros node, "listener" suscribes to the chatter topic, which is a type
    #of std_msgs.String. When new messages are received, callback is invoked with the messages as the first argument



    rospy.init_node('listener', anonymous=True) #anonymous = True was added bc of <2>

    rospy.Subscriber('chatter', String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

#In addition to the standard Python__main_check, this catches a rospy.ROSInterruptException exeception, which

#can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when ctrl +c is pressed or your node is otherwise shutdown



if __name__ == '__main__':
    listener()
