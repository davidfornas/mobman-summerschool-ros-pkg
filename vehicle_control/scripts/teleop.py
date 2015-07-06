#!/usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Code from: turtlebot_teleop

import rospy

from std_msgs.msg import Int32

import sys, select, termios, tty

msg = """
Control Your Vehicle!
---------------------------
Moving around:
q    w    e
a         d
     s    
z/x : increase/decrease max speeds by 10%
space key, k : force stop
anything else : stop smoothly
CTRL-C to quit
"""

moveBindings = {
    'w':(1,1),
    'a':(1,-1),
    's':(-1,-1),
    'd':(-1,1),
    'q':(1,0.4),
    'e':(0.4,1)
}

speedBindings={
    'x': 1.5,
    'z': 0.75
}
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 25

def vel(speed):
    return "currently:\tspeed %s" % (speed)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('vehicle_teleop')
    lpub = rospy.Publisher('/left_wheel_cmd', Int32, queue_size=5)
    rpub = rospy.Publisher('/right_wheel_cmd', Int32, queue_size=5)

    r = 0
    l = 0

    status = 0
    count = 0

    target_r_speed = 0
    target_l_speed = 0
    control_r_speed = 0
    control_l_speed = 0
    try:
        print msg
        while(1):
            key = getKey()
            #Move keys assign speed to wheels
            if key in moveBindings.keys():
                l = moveBindings[key][0]
                r = moveBindings[key][1]
                count = 0
            #Speed keys increase or decrease speed
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key]
                count = 0
                print vel(speed)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            #Stop keys set speed to zero
            elif key == ' ' or key == 'k' :
                r = 0
                l = 0
                control_r_speed = 0
                control_l_speed = 0
            else:
                count = count + 1
                if count > 4:
                    r = 0
                    l = 0
                if (key == '\x03'):
                    break

            acc=speed/10
            target_r_speed = speed * r
            target_l_speed = speed * l

            #Speed control with acceleration
            if target_l_speed > control_l_speed:
                control_l_speed = min( target_l_speed, control_l_speed + acc )
            elif target_l_speed < control_l_speed:
                control_l_speed = max( target_l_speed, control_l_speed - acc )
            else:
                control_l_speed = target_l_speed

            if target_r_speed > control_r_speed:
                control_r_speed = min( target_r_speed, control_r_speed + acc )
            elif target_r_speed < control_r_speed:
                control_r_speed = max( target_r_speed, control_r_speed - acc )
            else:
                control_r_speed = target_r_speed

            #Publish speeds
            lwheel = Int32()
            rwheel = Int32()

            lwheel.data = 90 + control_l_speed
            rwheel.data = 90 + control_r_speed

            rpub.publish(rwheel)
            lpub.publish(lwheel)

            #Debug
            print("loop: {0}".format(count))
            print("target: vl: {0}, wr: {1}".format(lwheel.data, rwheel.data))

    except:
        print e

    finally:

        lwheel = Int32()
        rwheel = Int32()

        lwheel.data = 0
        rwheel.data = 0

        rpub.publish(rwheel)
        lpub.publish(lwheel)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
