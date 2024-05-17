import sys, socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

def socket_init(ip, port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((ip, port))
    server_socket.settimeout(0.1)
    server_socket.listen(1)
    return server_socket

def socket_connect(server_socket):
    # wait for connect
    client_socket, client_address = server_socket.accept()
    print(f'client socket: {client_socket}, address: {client_address}')

    return client_socket

def socket_recv(client_socket):
    # wait for camera distance
    data = client_socket.recv(1024)
    if data:
        position = data.decode().split('/')[-2]
        print(f'position = {position}')
        return position
    print('get nothing')

def process(str_position):
    list_pos = str_position.split(' ')
    return list_pos[0], list_pos[1], list_pos[2]


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')      
        # publish data from camera
        self.publisher_ = self.create_publisher(PointStamped, 'camera_position', 10)


def main(args=None):

    # get args
    if args == None or len(args) == 1:
        print('The node needs socket ip and port')
        return -1
        
    ip, port = args[0], args[1]

    #init server socket
    server_socket = socket_init('192.168.150.132', 8888)

    # init Node
    rclpy.init(args=args)
    node = CameraNode()

    stop = False
    while not stop:
        client_socket = None
        try:
            # connect camera and host
            print('wait for connect...')
            client_socket = socket_connect(server_socket)
            print(f'client socket: {client_socket}, address: {client_address}')
        except TimeoutError:
            continue
        except Exception as e:
            print(f'An error occurred: {e}')
            break
            
        while True:
            try:
                # tcp wait recieve distance data
                position = socket_recv(client_socket)
                if not position:
                    print("distance not found, disconnected.")
                    client_socket.close()
                    break

                # processs position
                tx, ty, tz = process(position)
                
                # ros publish
                message = PointStamped()
                message.header.frame_id = 'odom'
                message.point.x = tx
                message.point.y = ty
                message.point.z = tz
                node.publisher_.publish(message)
            except KeyboardInterrupt as e:
                print(f'An error occurred: {e}')
                stop = True
                break
            except:
                print(f'An error occurred: {e}')
                break


    node.destroy_node()

    if server_socket:
        server_socket.close()

if __name__ == '__main__':
    main(sys.argv)

