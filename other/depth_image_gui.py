
from PyQt5 import QtWidgets, QtGui
import numpy as np
import cv2


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

    def update_image(self, data):

        height, width, cloud = data.height, data.width, data.cloud

        # set position
        self.setPosition(height, width, cloud)

        # set image
        image = np.asarray([point[2] for point in cloud]).astype(np.uint8)
        cv2.normalize(image, image, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        self.qImg = QtGui.QPixmap(QtGui.QImage(image, width, height, QtGui.QImage.Format_Grayscale8))
        self.image_frame.setPixmap(self.qImg)

    def setPosition(self, height, width, cloud):
        cloud.reshape(height, width, 3)
        self.cloud = cloud

    def get_clicked_position(self, event):
        # get pixel's position
        pos = event.pos()
        x, y, z = self.cloud[pos.y(), pos.x()]
        print('psoition mouse:', pos.x(), pos.y())
        print('position: xyz({}, {}, {});'.format(x, y, z))