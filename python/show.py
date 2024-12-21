import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import math
from PyQt5 import QtWidgets, QtCore

path = '/home/jaenote/rokey_week7_ws/python/openmanipulator_sub.URDF'
my_chain = ikpy.chain.Chain.from_urdf_file(path)
target_position = [ 0.2, 0, 0]
joints = my_chain.inverse_kinematics(target_position)
print("The angles of each joints are : ", joints)

import matplotlib.pyplot as plt
fig, ax = plot_utils.init_3d_figure()

th_delta = math.atan2(0.024, 0.128)
joints= [0, 1.57 - th_delta, -(1.57 - th_delta), 0, 0]
my_chain.plot(joints, ax, target=target_position)

ax.set_xlim(-0.2, 0.2)
class GUI:
    def __init__(self):
        self.app = QtWidgets.QApplication([])
        self.window = QtWidgets.QWidget()
        self.layout = QtWidgets.QVBoxLayout()
        self.window.setLayout(self.layout)
        
        self.label = QtWidgets.QLabel("Enter a number:")
        self.intput = QtWidgets.QLineEdit()
        self.layout.addWidget(self.intput)
        self.button = QtWidgets.QPushButton('Click me')
        self.button.clicked.connect(self.on_button_click)
        self.layout.addWidget(self.button)
        self.window.show()
        self.app.exec_()


    def on_button_click(self):
        text = self.intput.text() # (x, y, z)
        target_position = [float(x) for x in text.split(',')]
        path = '/home/jaenote/rokey_week7_ws/python/openmanipulator_sub.URDF'
        my_chain = ikpy.chain.Chain.from_urdf_file(path)
        self.ploting(target_position, my_chain)
    
    def ploting(self, target_position, my_chain):
        joints = my_chain.inverse_kinematics(target_position)
        fig, ax = plot_utils.init_3d_figure()
        my_chain.plot(joints, ax, target=target_position)
        ax.set_xlim(-0.2, 0.2)
        ax.set_ylim(-0.2, 0.2)
        ax.set_zlim(-0.2, 0.2)
        plt.show()
if __name__ == '__main__':

    GUI()


