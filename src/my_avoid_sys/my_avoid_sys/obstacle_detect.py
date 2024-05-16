import rclpy
from rclpy.node import Node
# msg
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
# for waiting 3 second
from threading import Thread
import time

class ObstacleDetector(Node):

    def __init__(self,):
        # name the node
        super().__init__('obstacle_detector')
        # edge
        self.imgSubscription = self.create_subscription(PointCloud2, '/camera/points', self.detect, 5)
        self.modePublisher = self.create_publisher(String, 'mode', 5)
        self.commandPublisher = self.create_publisher(String, 'command', 5)
        # mode
        self.mode = 0

    def detect(self, msg):
        assert isinstance(msg, PointCloud2)
        cloud = point_cloud2.read_points(msg,field_names=("x", "y", "z"))

        # mode 0: no obstacle => detecting
        if self.mode == 0:
            for point in cloud:
                # if too close => get conntrol and send stop command, set to mode 1
                if -0.075 < point[0] < 0.075 and point[1] < 0.1 and point[2] < 2:
                    self.modePublisher.publish(String(data='detector'))
                    self.commandPublisher.publish(String(data='detector k'))
                    thread = Thread(target=self.wait)
                    thread.start()
                    self.mode = 1
                    return
        
        # mode 1: wait until 3 second or obstacle leave
        elif self.mode == 1:
            # if obstacle leave => give back control, set to mode 0
            # else => until 3 second, set to mode 2
            for point in cloud:
                if -0.075 < point[0] < 0.075 and point[1] < 0.1 and point[2] < 2:
                    return
            self.modePublisher.publish(String(data='teleop'))
            self.mode = 0
        
        # mode 2: turn left until no obstacle and give back control, set to mode 0
        elif self.mode == 2:
            for point in cloud:
                if -0.075 < point[0] < 0.075 and point[1] < 0.1 and point[2] < 2:
                    self.commandPublisher.publish(String(data='detector j'))
                    return
            self.commandPublisher.publish(String(data='detector k'))
            self.modePublisher.publish(String(data='teleop'))
            self.mode = 0

    def wait(self):
        # wait 3 second
        time.sleep(3)
        # if obstacle dont leave => turn left
        if self.mode == 1:
            self.mode = 2

def main():
    print('start detect')
    rclpy.init()
    # start node
    detector = ObstacleDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print('[KeyboardInterrupt]')
    finally:
        # avoid deadlock
        detector.commandPublisher.publish(String(data='detector k'))
        detector.modePublisher.publish(String(data='teleop'))
        # stop node
        detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()