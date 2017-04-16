#!/usr/bin/env python 
##Above: Every Python ROS node will have this declaration at the top.
###The first linemakes sure your script is executed as a python script


## Simple talker demo that published std_msgs/Strings messages to the 'chatter' topic

import rospy #Import rospy if you are writing a ROS Node
from std_msgs.msg import String #so we can reuse std_msgs/String message type (A simple string container) for publishing

def talker(): #Creates Ros publish object publisher. So this section of code defines talker's interface with the rest of ROS
    pub = rospy.Publisher('chatter', String, queue_size=10) # declares your node is publishing to the chatter topic using message type String. String here is actually a class: std_msgs.msg.String. Queze size argument limits the amount of queded messages if any subscriber is not receiving them fast enough
    rospy.init_node('talker', anonymous=True) #Very important. It tells rospy the name of your node. Once rospy has this information, it can communicate with ROS master. In this case our node will be callked 'talker'
    rate = rospy.Rate(2) # 10hz #This line creates a Rate object rate with the help of its method sleep(), it offers a convient way for looping at a desired rate. With its argument of 10, we should expect to go through the loop 10 times per second
    
    #This loop checks rospy.is_shutdown() flag and then does work. you have to check is_shutdown to check if your program should exit (If there is a Ctrl+C or otherwise). 
    #In this case, "work" is a call to pub.publish(String(str)) that publishes our chatter topic using a newly created String message
    
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time() #define your message
        rospy.loginfo(hello_str) #This loop also calls this function which performs a triple duty: messages get printed to screen, gets written into a log file, and gets written to rosout. Rosout is a handy debugging tool. Youcan pull up messages using rqt_console instead of having to find the console window with your Node's output.
        pub.publish(hello_str) #publish your message by calling the pub.publish(String(str)) function. This publishes our chatter topic using a newly created String message
        rate.sleep() #The loop calls this function, which sleeps just logn enough to maintain the desired rate through the loop
#std_msgs.msg.String is a very simple message type, so you may be wondering what it looks like to publish more complicated types. The general rule of thumb is that constructor args are in the same order as in the .msg file. You can also pass in no arguments and initialize the fields directly, e.g.
#msg = String()
#msg.data = str
#or you can initialize some of the fields and leave the rest with default values:
#String(data=str)


#In addition to the standard Python__main_check, this catches a rospy.ROSInterruptException exeception, which
#can be thrown by rospy.sleep() and rospy.Rate.sleep() methods when ctrl +c is pressed or your node is otherwise shutdown

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
