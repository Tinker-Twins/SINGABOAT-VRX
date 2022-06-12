#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Empty
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

################################################################################

THRUST_LIMIT = 1
THRUST_STEP = 0.2

info = """
-------------------------------------
SingaBoat - WAM-V Teleoperation Panel
-------------------------------------

             Q   W   E
             A   S   D
             Z   X   C

W/S : Increase/decrease surge rate
A/D : Increase/decrease sway rate
Q/E : Increase/decrease yaw rate
X   : Apply hydrodynamic brakes
Z   : Shoot the projectile
C   : Release all controls

Press CTRL+C to quit

NOTE: Press keys within this terminal
-------------------------------------
"""

error = """
ERROR: Communication failed!
"""

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def boundThrust(trust_cmd):
    trust_cmd = constrain(trust_cmd, -THRUST_LIMIT, THRUST_LIMIT)
    return trust_cmd

################################################################################

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('singaboat_teleop_key')

    front_left_thruster_pub = rospy.Publisher('/wamv/thrusters/front_left_thruster_thrust_cmd', Float32, queue_size=10)
    front_right_thruster_pub = rospy.Publisher('/wamv/thrusters/front_right_thruster_thrust_cmd', Float32, queue_size=10)
    rear_left_thruster_pub = rospy.Publisher('/wamv/thrusters/rear_left_thruster_thrust_cmd', Float32, queue_size=10)
    rear_right_thruster_pub = rospy.Publisher('/wamv/thrusters/rear_right_thruster_thrust_cmd', Float32, queue_size=10)
    ball_shooter_pub = rospy.Publisher("/wamv/shooters/ball_shooter/fire", Empty, queue_size=1)

    front_left_thruster_thrust_cmd = 0.0
    front_right_thruster_thrust_cmd = 0.0
    rear_left_thruster_thrust_cmd = 0.0
    rear_right_thruster_thrust_cmd = 0.0

    try:
        print(info)

        while(1):
            key = getKey()
            if key == 'w' :
                front_left_thruster_thrust_cmd = 0.0
                front_right_thruster_thrust_cmd = 0.0
                rear_left_thruster_thrust_cmd = boundThrust(rear_left_thruster_thrust_cmd + THRUST_STEP)
                rear_right_thruster_thrust_cmd = boundThrust(rear_right_thruster_thrust_cmd + THRUST_STEP)
            elif key == 's' :
                front_left_thruster_thrust_cmd = boundThrust(front_left_thruster_thrust_cmd + THRUST_STEP)
                front_right_thruster_thrust_cmd = boundThrust(front_right_thruster_thrust_cmd + THRUST_STEP)
                rear_left_thruster_thrust_cmd = 0.0
                rear_right_thruster_thrust_cmd = 0.0
            elif key == 'a' :
                front_left_thruster_thrust_cmd = 0.0
                front_right_thruster_thrust_cmd = boundThrust(front_right_thruster_thrust_cmd + THRUST_STEP/2)
                rear_left_thruster_thrust_cmd = 0.0
                rear_right_thruster_thrust_cmd = boundThrust(rear_right_thruster_thrust_cmd + THRUST_STEP/2)
            elif key == 'd' :
                front_left_thruster_thrust_cmd = boundThrust(front_left_thruster_thrust_cmd + THRUST_STEP/2)
                front_right_thruster_thrust_cmd = 0.0
                rear_left_thruster_thrust_cmd = boundThrust(rear_left_thruster_thrust_cmd + THRUST_STEP/2)
                rear_right_thruster_thrust_cmd = 0.0
            elif key == 'q' :
                front_left_thruster_thrust_cmd = boundThrust(front_left_thruster_thrust_cmd - THRUST_STEP)
                front_right_thruster_thrust_cmd = boundThrust(front_right_thruster_thrust_cmd + THRUST_STEP)
                rear_left_thruster_thrust_cmd = boundThrust(rear_left_thruster_thrust_cmd + THRUST_STEP)
                rear_right_thruster_thrust_cmd = boundThrust(rear_right_thruster_thrust_cmd - THRUST_STEP)
            elif key == 'e' :
                front_left_thruster_thrust_cmd = boundThrust(front_left_thruster_thrust_cmd + THRUST_STEP)
                front_right_thruster_thrust_cmd = boundThrust(front_right_thruster_thrust_cmd - THRUST_STEP)
                rear_left_thruster_thrust_cmd = boundThrust(rear_left_thruster_thrust_cmd - THRUST_STEP)
                rear_right_thruster_thrust_cmd = boundThrust(rear_right_thruster_thrust_cmd + THRUST_STEP)
            elif key == 'x' :
                front_left_thruster_thrust_cmd = 1.0
                front_right_thruster_thrust_cmd = 1.0
                rear_left_thruster_thrust_cmd = 1.0
                rear_right_thruster_thrust_cmd = 1.0
            elif key == 'z' :
                ball_shooter_pub.publish()
            elif key == 'c' :
                front_left_thruster_thrust_cmd = 0.0
                front_right_thruster_thrust_cmd = 0.0
                rear_left_thruster_thrust_cmd = 0.0
                rear_right_thruster_thrust_cmd = 0.0
            else:
                if (key == '\x03'): # CTRL+C
                    break

            front_left_thruster_pub.publish(front_left_thruster_thrust_cmd)
            front_right_thruster_pub.publish(front_right_thruster_thrust_cmd)
            rear_left_thruster_pub.publish(rear_left_thruster_thrust_cmd)
            rear_right_thruster_pub.publish(rear_right_thruster_thrust_cmd)

    except:
        print(error)

    finally:
        front_left_thruster_thrust_cmd = 0.0
        front_right_thruster_thrust_cmd = 0.0
        rear_left_thruster_thrust_cmd = 0.0
        rear_right_thruster_thrust_cmd = 0.0

        front_left_thruster_pub.publish(front_left_thruster_thrust_cmd)
        front_right_thruster_pub.publish(front_right_thruster_thrust_cmd)
        rear_left_thruster_pub.publish(rear_left_thruster_thrust_cmd)
        rear_right_thruster_pub.publish(rear_right_thruster_thrust_cmd)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
