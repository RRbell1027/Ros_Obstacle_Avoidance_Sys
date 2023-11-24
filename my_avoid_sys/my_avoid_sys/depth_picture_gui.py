import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from PyQt5 import QtWidgets, QtGui
import sys

from threading import Thread


class DepthCollector(Node):

    def __init__(self):

        # Name the node
        super().__init__('depth_picture_gui')

        # Create '/camera/points/ subscriber
        self.subscription = self.create_subscription(
                 PointCloud2,
                '/camera/points',
                self.callback,
                5)

        self.gui = DisplayImageWidget()

    def callback(self, msg):
        
        # Security of interface type
        assert isinstance(msg, PointCloud2)

        # Read data
        height = msg.height
        width = msg.width
        cloud = point_cloud2.read_points(msg,field_names=("x", "y", "z"))
        cloud.resize(height, width)
        img = point_cloud2.read_points(msg,field_names=("z"))
        img.resize(height, width)

        # Show image
        self.gui.setImage(img)
        self.gui.setPosition(cloud)

class DisplayImageWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # configuration of window
        self.image_frame = QtWidgets.QLabel()
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

        # change mouse press event
        self.image_frame.mousePressEvent = self.get_clicked_position

    def setImage(self, img):
        # fix type
        img = img.astype(np.uint8)
        # normalize to 0 - 255 (fix gray scale)
        cv2.normalize(img, img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        # set image
        self.qImg = QtGui.QPixmap(QtGui.QImage(img.data, img.shape[1], img.shape[0], QtGui.QImage.Format_Grayscale8))
        self.image_frame.setPixmap(self.qImg)

    def setPosition(self, cloud):
        self.cloud = cloud

    def get_clicked_position(self, event):
        # get pixel's position
        pos = event.pos()
        x, y, z = self.cloud[pos.y(), pos.x()]
        print('psoition mouse:', pos.x(), pos.y())
        print('position: xyz({}, {}, {});'.format(x, y, z))

def main(args=None):
    rclpy.init(args=args)
    
    app = QtWidgets.QApplication(sys.argv)

    # implement node
    executor = MultiThreadedExecutor()
    collector = DepthCollector()
    executor.add_node(collector)
    
    # thread
    thread = Thread(target=executor.spin)
    thread.start()

    try:
        collector.gui.show()
        sys.exit(app.exec())

    finally:
        collector.destroy_node()
        executor.shutdown()

if __name__ == '__main__':
    main()