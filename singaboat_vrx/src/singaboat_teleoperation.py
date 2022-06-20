#!/usr/bin/env python3

import sys, select, os
if os.name == 'nt':
    import msvcrt
else:
    import tty, termios
import rospy
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
from std_msgs.msg import Empty
from singaboat_vrx.cfg import TeleoperationConfig
from singaboat_vrx.common_utilities import constrain

################################################################################

info = """
-------------------------------------
SINGABOAT - WAM-V Teleoperation Panel
-------------------------------------

             Q   W   E
             A   S   D
             Z       C

W/S : Increase/decrease surge rate
A/D : Increase/decrease sway rate
Q/E : Increase/decrease yaw rate
Z   : Shoot the projectile
C   : Release all controls

Press CTRL+C to quit

NOTE: Press keys within this terminal
-------------------------------------
"""

error = """
ERROR: Communication failed!
"""

def get_key():
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

def config_callback(config, level):
    return config

################################################################################

if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    # Teleoperation node
    rospy.init_node('singaboat_teleoperation')

    # Dynamic reconfigure server
    dyn_reconf_srv = Server(TeleoperationConfig, config_callback)

    # Publishers
    center_thruster_pub = rospy.Publisher('/wamv/thrusters/center_thruster_thrust_cmd', Float32, queue_size = 10)
    left_thruster_pub   = rospy.Publisher('/wamv/thrusters/left_thruster_thrust_cmd', Float32, queue_size = 10)
    right_thruster_pub  = rospy.Publisher('/wamv/thrusters/right_thruster_thrust_cmd', Float32, queue_size = 10)
    ball_shooter_pub    = rospy.Publisher("/wamv/shooters/ball_shooter/fire", Empty, queue_size=1)

    THRUST_LIMIT = dyn_reconf_srv.config['thrust_limit'] # Maximum permitted thrust command for ASV
    THRUST_STEP  = dyn_reconf_srv.config['thrust_step'] # Stepping amount to alter thrust command for ASV

    center_thrust_cmd = 0.0
    left_thrust_cmd   = 0.0
    right_thrust_cmd  = 0.0

    try:
        print(info)

        while(1):
            key = get_key()
            if key == 'w' :
                center_thrust_cmd = 0.0
                left_thrust_cmd   = constrain((left_thrust_cmd + THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
                right_thrust_cmd  = constrain((right_thrust_cmd + THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
            elif key == 's' :
                center_thrust_cmd = 0.0
                left_thrust_cmd   = constrain((left_thrust_cmd - THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
                right_thrust_cmd  = constrain((right_thrust_cmd - THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
            elif key == 'a' :
                center_thrust_cmd = constrain((center_thrust_cmd + THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
                left_thrust_cmd   = 0.0
                right_thrust_cmd  = 0.0
            elif key == 'd' :
                center_thrust_cmd = constrain((center_thrust_cmd - THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
                left_thrust_cmd   = 0.0
                right_thrust_cmd  = 0.0
            elif key == 'q' :
                center_thrust_cmd = 0.0
                left_thrust_cmd   = constrain((left_thrust_cmd - THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
                right_thrust_cmd  = constrain((right_thrust_cmd + THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
            elif key == 'e' :
                center_thrust_cmd = 0.0
                left_thrust_cmd   = constrain((left_thrust_cmd + THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
                right_thrust_cmd  = constrain((right_thrust_cmd - THRUST_STEP), -THRUST_LIMIT, THRUST_LIMIT)
            elif key == 'z' :
                ball_shooter_pub.publish()
            elif key == 'c' :
                center_thrust_cmd = 0.0
                left_thrust_cmd   = 0.0
                right_thrust_cmd  = 0.0
            else:
                if (key == '\x03'): # CTRL+C
                    break

            center_thruster_pub.publish(center_thrust_cmd)
            left_thruster_pub.publish(left_thrust_cmd)
            right_thruster_pub.publish(right_thrust_cmd)

    except:
        print(error)

    finally:
        center_thrust_cmd = 0.0
        left_thrust_cmd   = 0.0
        right_thrust_cmd  = 0.0

        center_thruster_pub.publish(center_thrust_cmd)
        left_thruster_pub.publish(left_thrust_cmd)
        right_thruster_pub.publish(right_thrust_cmd)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
