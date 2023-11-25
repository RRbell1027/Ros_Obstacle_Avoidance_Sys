import rclpy
from rclpy.node import Node

# msg
from std_msgs.msg import String
from geometry_msgs.msg import Twist

# dictionary of movement
moveBindings = {
    'u': (1, 0, 0, 1),
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'k': (0, 0, 0, 0),
    'l': (0, 0, 0, -1),
    'm': (-1, 0, 0, -1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
}

# dictionary of speed change
speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    'w': (1.1, 1),
    'x': (.9, 1),
    'e': (1, 1.1),
    'c': (1, .9),
}

def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)

class Multiplexing(Node):
    def __init__(self):
        # name the node
        super().__init__('robot_controller')      
        # mode
        self.mode = 'teleop'
        # value
        self.speed = 0.5
        self.turn = 0.1
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.status = 0.0
        # edges
        self.commandListener = self.create_subscription(String,  'command', self.exeCommand, 5)
        self.modeListener = self.create_subscription(String, 'mode', self.setMode, 5)
        self.commandPublsher = self.create_publisher(Twist, 'cmd_vel', 5)
        
    def exeCommand(self, msg):
        assert isinstance(msg, String)
        # print msg
        print('[robot_controller]receive command:', msg.data)
        # msg processing
        author, key = msg.data.split(' ')
        # check current author
        if author == self.mode:
            # key processing
            if key in moveBindings.keys():
                self.x = moveBindings[key][0]
                self.y = moveBindings[key][1]
                self.z = moveBindings[key][2]
                self.th = moveBindings[key][3]
            elif key in speedBindings.keys():
                self.speed = self.speed * speedBindings[key][0]
                self.turn = self.turn * speedBindings[key][1]
                print(vels(self.speed, self.turn))
            # publish twist
            twist = Twist()
            twist.linear.x = self.x * self.speed
            twist.linear.y = self.y * self.speed
            twist.linear.z = self.z * self.speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = self.th * self.turn
            self.commandPublsher.publish(twist)

    def setMode(self, msg):
        assert isinstance(msg, String)
        # print
        print('[robot_controller]set mode:', msg.data)
        # set mode
        self.mode = msg.data

def main():
    print('[robot_controller]controller start')
    rclpy.init()
    # start node
    multiplexing = Multiplexing()
    try:
        rclpy.spin(multiplexing)
    except KeyboardInterrupt:
        print('[robot_controller]KeyboardInterrupt')
    finally:
        # stop node
        multiplexing.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()