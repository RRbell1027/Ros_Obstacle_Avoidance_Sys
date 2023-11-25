import sys
import socket
import pickle
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal, QObject
import threading

from depth_image_gui import DisplayImageWidget

class Image:
    def __init__(self, height, width, cloud):
        self.height = height
        self.width = width
        self.cloud = cloud

class SocketThread(QObject):
    image_received = pyqtSignal(object)

    def __init__(self, host_ip, host_port):
        super().__init__()
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((host_ip, host_port))
        self.flag = False

    def run(self):
        while True:
            # Send request to server
            self.client_socket.send('send_image'.encode())

            # Receive image info
            size_info = self.client_socket.recv(4)

            if size_info:

                image_size = int.from_bytes(size_info, 'big')

                # Receive image data
                image_data = b''
                while len(image_data) < image_size:
                    data = self.client_socket.recv(4096)
                    image_data += data
                received_image = pickle.loads(image_data)
                
                # Emit signal with the received image
                self.image_received.emit(received_image)

            if self.flag:
                self.client_socket.send('close'.encode())
                self.client_socket.close()

    def close(self):
        self.flag = True

def start_socket_thread(socket_thread):
    socket_thread.run()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    # Create and show the GUIs
    display_widget = DisplayImageWidget()
    display_widget.show()

    # Create and start the socket thread
    host_ip = '127.0.0.1'
    host_port = 9999
    socket_thread = SocketThread(host_ip, host_port)
    socket_thread.image_received.connect(display_widget.update_image)

    # Start the socket thread in a separate thread
    socket_thread_thread = threading.Thread(target=start_socket_thread, args=(socket_thread,))
    socket_thread_thread.start()

    app.exec_()
    socket_thread.close()
    sys.exit()