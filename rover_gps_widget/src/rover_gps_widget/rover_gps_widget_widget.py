"""Rovus -- GPS Widget"""

# import os
# import rospkg
# import rospy
# from rospkg import RosPack
# from python_qt_binding import loadUi
# from PyQt5 import QtGui
# from PyQt5 import QtCore
# from PyQt5.QtCore import QObject
from PyQt5 import QtWidgets
from PyQt5.QtCore import Qt
# from PyQt5.QtGui import QKeySequence
# from PyQt5.QtWidgets import QShortcut, QSlider, QAbstractSlider, QSpinBox
# from rover_udes.msg import CamCommand
# from std_msgs.msg import Bool
# from std_srvs.srv import SetBool
from rover_gps_widget.ui_gps_widget import Ui_gps_widget


class RoverGpsWidgetWidget(QtWidgets.QWidget):
    """Rover GPS Widget"""
    def __init__(self):
        super(RoverGpsWidgetWidget, self).__init__()

        self.ui = Ui_gps_widget()
        self.ui.setupUi(self)
        self.setObjectName('RoverGpsWidgetWidget')
        self.editable_fields = [
            self.ui.doubleSpinBox_currentLat,
            self.ui.doubleSpinBox_currentLong,
            self.ui.lineEdit_currentCoord,
        ]

        # self.move_base = rospy.Publisher('move_base', , queue_size=10)
        # rospy.Timer(rospy.Duration(1.0/10.0), self.publish_command)

        self.connect_items()

    def connect_items(self):
        """Connect all UI elements"""
        self.ui.checkBox_edit.stateChanged.connect(self.edit_changed)

    def edit_changed(self, state):
        """State of Edit checkbox changed callback"""
        if state == Qt.Checked:
            self.edit_checked()
        else:
            self.edit_unchecked()

    def edit_checked(self):
        """Edit changed to checked"""
        #self.windowTitle.setText('Ah')
        # self.ui.doubleSpinBox_currentDistance.setValue(1)
        for field in self.editable_fields:
            field.setReadOnly(True)

    def edit_unchecked(self):
        """Edit changed to unchecked"""
        #self.windowTitle.setText('Oh')
        # self.ui.doubleSpinBox_currentDistance.setValue(2)
        for field in self.editable_fields:
            field.setReadOnly(False)
