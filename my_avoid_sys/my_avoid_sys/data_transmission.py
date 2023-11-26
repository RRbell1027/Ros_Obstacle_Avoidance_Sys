import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from threading import Thread
import socket, pickle
from datetime import datetime
import time

# data structure for reading PointCloud2
class Image:
    def __init__(self, t, h, w, c):
        self.time = t
        self.height = h
        self.width = w
        self.cloud = c

class DepthCameraImage(Node):
    def __init__(self):
        super().__init__('data_server')
        self.camera_subsciption = self.create_subscription(PointCloud2, '/camera/points', self.depth_camera_callback, 5)

    # save image
    def depth_camera_callback(self, msg):
        assert isinstance(msg, PointCloud2)
        self.image = Image(
            datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            msg.height, 
            msg.width,
            point_cloud2.read_points_numpy(msg,field_names=("x", "y", "z"))
        )

def main():

    # ros node setting
    rclpy.init()
    node = DepthCameraImage()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    thread = Thread(target=executor.spin)
    thread.start()

    # socker clinet setting
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('127.0.0.1', 9999))

    client_socket.send('00000001'.encode())
    # receive request
    print('attempt to connect to server')
    time.sleep(5)

    try:
        while True:
            # wait for request
            request = client_socket.recv(1024).decode()

            # request for image
            if request == 'send_image':
                
                image_to_send = node.image
                image_bytes = pickle.dumps(image_to_send)

                # image info
                print('size =', len(image_bytes))
                size_info = len(image_bytes).to_bytes(4, 'big')
                client_socket.send(size_info)

                for i in range(0, len(image_bytes), 4096):
                    chunk = image_bytes[i: min(i+4096, len(image_bytes))]
                    client_socket.send(chunk)
                print('Image sent seccessfully')

            if request == 'close':
                client_socket.close()

    finally:

        client_socket.close()

        node.destroy_node()
        executor.shutdown()