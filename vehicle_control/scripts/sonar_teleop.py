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

# COde from: turtlebot_teleop

import rospy

from std_msgs.msg import Int32

import sys, select, termios, tty

sonar_value=0

msg = """

Control The Sonar!
---------------------------
h Move 10 deg. left
j Move to 0 deg
k Move 10 deg right

CTRL-C to quit
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def pos(p):
    return "currently:\t%s degrees" % (p)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    rospy.init_node('sonar_teleop')
    pub = rospy.Publisher('/sonar_servo_cmd', Int32, queue_size=5)

    status = 90
    try:
        print msg
        print pos(status)
        while(1):
            key = getKey()
            if key == 'h':
                status = status - 10
                if( status < 0 ): 
                    status = 0
            elif key == 'j' or key == ' ':
                status = 90
            elif key == 'k' :
                status = status + 10
                if( status > 180 ):
                    status = 180
            elif (key == '\x03'):
                break

            print pos(status)
            
            servo = Int32()
            servo.data = status            
            pub.publish(servo)

            #print("servo pos: {0}".format(servo))

    except:
        print e

    finally:

        servo = Int32()
        servo.data = 0
            
        pub.publish(servo)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
