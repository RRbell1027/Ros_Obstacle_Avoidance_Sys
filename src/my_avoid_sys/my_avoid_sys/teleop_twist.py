import sys

import rclpy

from std_msgs.msg import String

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


msg = """
This node takes keypresses from the keyboard and publishes them
as Twist messages. It works best with a US keyboard layout.
------------------------------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
------------------------------------------------------
CTRL-C to quit
"""

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main():
    # get os setting
    settings = saveTerminalSettings()
    # init executable
    rclpy.init()
    # create node
    node = rclpy.create_node('my_teleop_twist')
    pub = node.create_publisher(String, 'command', 10)
    # input
    try:
        print(msg)
        while True:
            # get input value
            key = getKey(settings)
            # quit if CTRL-C
            if (key == '\x03'):
                break
            # create command
            command = String(data='teleop {}'.format(key))
            # send command
            pub.publish(command)
    except Exception as e:
        print(e)
    finally:
        # send stop key
        command = String(data='teleop k')
        pub.publish(command)
        # restore os setting
        restoreTerminalSettings(settings)


if __name__ == '__main__':
    main()