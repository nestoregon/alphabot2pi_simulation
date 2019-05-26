#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import time

import sys, select, termios, tty

msg = """
Reading from the keyboard and publishing to /remote_state and /remote_key
---------------------------
/remote_key:

        w
    a   s   d

    w: forward
    a: left
    s: backward
    d: right
    (space bar to stop)

/remote_state
    1: manual
    2: ball_following_drive
    3: ball_following_servo

Close Window to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    n = 0
    rospy.init_node("remote_node", anonymous=False)
    pubDirection = rospy.Publisher("/remote_key", String, queue_size=5)
    pubState = rospy.Publisher("/remote_state", String, queue_size=5)


    while not rospy.is_shutdown():
        key = getKey()
        print(key)
        if(key != None):
            n = 0
            if key == '1':
                pubState.publish("manual")
            if key == '2':
                pubState.publish("ball_following_drive")
            if key == '3':
                pubState.publish("ball_following_servo")
            if key == 'w':
                pubDirection.publish("forward")
            if key == 'a':
                pubDirection.publish("left")
            if key == ' ':
                pubDirection.publish("stop")
            if key == 'd':
                pubDirection.publish("right")
            if key == 's':
                pubDirection.publish("backward")
            if key == 's':
                exit
        else:
            n += 1
            if n > 20000:
                n = 0
                # pub.publish("stop")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
