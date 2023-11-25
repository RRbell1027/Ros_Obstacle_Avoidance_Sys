import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from threading import Thread
import socket, pickle

# data structure for reading PointCloud2
class Image:
    def __init__(self, h, w, c):
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

    # socker server setting
    bind_ip = '127.0.0.1'
    bind_port = 9999
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((bind_ip, bind_port))
    server_socket.listen(5)

    # waitng for connect
    client_socket, addr = server_socket.accept()
    print('Connected by', addr)

    try:
        while True:
            # receive request
            request = client_socket.recv(1024).decode()
            print('client:', addr, 'send request:', request)

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
        node.destroy_node()
        executor.shutdown()