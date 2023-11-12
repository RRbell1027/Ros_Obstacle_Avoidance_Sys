import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import cv2

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from PyQt5 import QtWidgets, QtGui, QtCore
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
        # cloud = point_cloud2.read_points(msg,field_names=("x","y","z"))
        cloud = point_cloud2.read_points(msg,field_names=("z"))
        img = cloud.astype(np.uint8)
        img.resize(height, width)
        cv2.normalize(img, img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Convert to grayscale array
        # img = np.zeros([height, width])
        # for i in range(height):
        #     for j in range(width):
        #         img[i, j] = cloud[i * width + j][2]
        # cv2.normalize(img, img, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Show image
        self.gui.setImage(img)

class DisplayImageWidget(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # configuration of window
        self.image_frame = QtWidgets.QLabel()
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

    def setImage(self, img):
        self.qImg = QtGui.QPixmap(QtGui.QImage(img.data, img.shape[1], img.shape[0], QtGui.QImage.Format_Grayscale8))
        self.image_frame.setPixmap(self.qImg)

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