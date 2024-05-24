import sys, socket, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster

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

    return client_socket, client_address

def socket_recv(client_socket):
    # wait for camera distance
    data = client_socket.recv(1024)
    if data:
        data = data.decode().split('/')[-2]
        return data
    print('get nothing')

def process(data):
    data = data.split(' ')
    translation = [float(data[0]), float(data[1]), float(data[2])]
    orientation = [float(data[3]), float(data[4]), float(data[5]), float(data[6])]
    acceleration = [float(data[7]), float(data[8])]
    a_velocity = [float(data[9])]
    return translation, orientation, acceleration, a_velocity


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.last_time = None
        self.velocity_x = 0.0
        self.velocity_y = 0.0
        self.position_x = 0.0
        self.position_y = 0.0

    def update_odom(self, translation, orientation, acceleration, a_velocity):
        # get time
        current_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # calculate movement
        self.velocity_x += acceleration[0] * dt
        self.velocity_y += acceleration[1] * dt 

        # publish tf topic
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]

        self.tf_broadcaster.sendTransform(transform)

        # publish odom topic
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        odom.pose.pose.position.x = translation[0]
        odom.pose.pose.position.y = translation[1]
        odom.pose.pose.position.z = translation[2]
        odom.pose.pose.orientation.x = orientation[0]
        odom.pose.pose.orientation.y = orientation[1]
        odom.pose.pose.orientation.z = orientation[2]
        odom.pose.pose.orientation.w = orientation[3]
        
        odom.twist.twist.linear.x = self.velocity_x
        odom.twist.twist.linear.y = self.velocity_y
        odom.twist.twist.angular.z = self.a_velocity[0]
        
        self.odom_pub.publish(odom)
        self.last_time = current_time


def main(args=None):

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
            client_socket, client_address = socket_connect(server_socket)
            print(f'client socket: {client_socket}, address: {client_address}')
        except TimeoutError:
            continue
        except Exception as e:
            print(f'An error occurred: {e}')
            break
            
        while True:
            try:
                # tcp wait recieve distance data
                data = socket_recv(client_socket)
                if not data:
                    print("distance not found, disconnected.")
                    client_socket.close()
                    break

                # processs position
                translation, orientation, acceleration, a_velocity = process(data)

                # ros publish
                node.update_odom(translation, orientation, acceleration, a_velocity)

            except KeyboardInterrupt as e:
                print(f'An error occurred: {e}')
                stop = True
                break
            except Exception as e:
                print(f'An error occurred: {e}')
                break


    node.destroy_node()

    if server_socket:
        server_socket.close()

if __name__ == '__main__':
    main(sys.argv)

