import socket
import pickle
import threading
import db_manager

# for displaying depth images
import sys
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import pyqtSignal, QThread
from depth_image_gui import DisplayImageWidget

# data structure for reading PointCloud2
class Image:
    def __init__(self, t, h, w, c):
        self.time = t
        self.height = h
        self.width = w
        self.cloud = c


class ClientsListener(QThread):

    images_received = pyqtSignal(object)

    def __init__(self, client_socket):
        super().__init__()
        # set client socket
        self.client_socket = client_socket

        # # display image
        # self.display_widget = DisplayImageWidget()
        # self.display_widget.show()

        self.images_received.connect(display_widget.update_image)

    def run(self):
        # receive identity
        identity = self.client_socket.recv(1024).decode()
        print('Received identity:', identity)

        # add identity into database if not exist
        if db_manager.is_car_id_exists(identity):
            print('Create new car:', identity)
            db_manager.add_car(identity)
        while True:
    
            # send image request
            # print('send image request')
            self.client_socket.send('send_image'.encode())

            # Receive image info
            size_info = self.client_socket.recv(4)
            image_size = int.from_bytes(size_info, 'big')
            # print('image size =', image_size)

            # receive image data package
            image_data = b''
            while len(image_data) < image_size:
                data = self.client_socket.recv(4096)
                image_data += data
            received_image = pickle.loads(image_data)
            # print('load data sucessfully')

            # change display image
            self.images_received.emit(received_image)
            # print('iamge change')

            # insert image into database
            db_manager.add_data(identity, received_image.time, received_image.cloud)
            # print('database insert')

        # except Exception as e:
        #     print(e)
        #     # disconnect
        #     self.client_socket.close()


def listen_for_clients():
    while True:
        client_socket, addr = server_socket.accept()
        print('Connected by', addr)

        # provide each client with 'handle_client' thread
        listener = ClientsListener(client_socket)
        listener.start()


if __name__ == '__main__':
    app = QApplication(sys.argv)

    # socket setting
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('127.0.0.1', 9999))
    server_socket.listen(5)

    # display image
    display_widget = DisplayImageWidget()
    display_widget.show()

    # start assigning thread to clients
    client_listener_thread = threading.Thread(target=listen_for_clients)
    client_listener_thread.start()

    sys.exit(app.exec_())