from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import sys
import typing
import rospkg
import rospy
import rosservice
from python_qt_binding import loadUi
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QPixmap
from PyQt5.QtWidgets import QPushButton, QComboBox, QApplication, QDoubleSpinBox, QLineEdit, QWidget, QFileDialog, QGraphicsScene
from robotnik_msgs.msg import inputs_outputs
from robotnik_msgs.srv import set_digital_output
from std_srvs.srv import SetBool
from datetime import datetime
from threading import Lock
import math
from rover_control_msgs.msg import gps

class RoverMapGuiWidget(QtWidgets.QWidget):
    def __init__(self):
        self.name: str = "RoverMapGuiWidget"
        super(RoverMapGuiWidget, self).__init__()

        # Locate and load the UI file for the RoverMap GUI
        ui_file = os.path.join(rospkg.RosPack().get_path('rover_map_gui'), 'resource', 'rover_map_gui.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverMapGuiWidget')

        # Paths to UI and resource files
        ui_file = os.path.join(rospkg.RosPack().get_path('rover_map_gui'), 'resource', 'rover_map_gui.ui')
        map_image_path = os.path.join(rospkg.RosPack().get_path('rover_map_gui'), 'resource', 'studio_map.png')
        rover_logo_path = os.path.join(rospkg.RosPack().get_path('rover_map_gui'), 'resource', 'location_icon.png')
        loadUi(ui_file, self)
        self.setObjectName('RoverMapGuiWidget')

        self.lb_rover_icon: QPushButton

        # Initialize the graphics scene
        self.scene: QGraphicsScene = QGraphicsScene(self)

        # Load the rover icon with the size and position
        self.rover_logo_pixmap: QPixmap = QPixmap(rover_logo_path).scaled(40, 40, QtCore.Qt.KeepAspectRatio)
        #pixmap_resized = pixmap.scaled(720, 405, QtCore.Qt.KeepAspectRatio)

        self.map_image = self.scene.addPixmap(QPixmap(map_image_path))
        #self.rover_icon = self.scene.addPixmap(self.rover_logo_pixmap)

        # Set the QPixmap to QLabel
        self.lb_rover_icon.setPixmap(self.rover_logo_pixmap)
        self.lb_rover_icon.move(400, 350)
        self.lb_rover_icon.setFixedWidth(600)
        self.lb_rover_icon.setFixedHeight(600)
        #self.rover_icon.hide()  # hide until we have GPS data

        #top-left reference point
        self.min_coord_x = float(0.0)
        self.min_coord_y = float(0.0)
        self.min_coord_lon = float(45.22443)
        self.min_coord_lat = float(-71.55367)

        self.earth_radius = 6371

        #bottom-right reference point
        self.max_coord_x = self.rover_logo_pixmap.width()
        self.max_coord_y = self.rover_logo_pixmap.height()
        self.max_coord_lon = 45.376675
        self.max_coord_lat = -71.923212
        
        self.lock_position: Lock = Lock()
        with self.lock_position:
            self.current_latitude: float = -690.0
            self.current_longitude: float = -690.0
            self.current_height: float = -690.0
            self.current_heading: float = -690.0

        self.gv_map.setScene(self.scene)

        ## Subscribe to the GPS data topic
        self.sub_gps = rospy.Subscriber("/gps_data", gps, self.cbGPSData)
        # Set up a timer to update the rover's position on the map
        self.position_updater = rospy.Timer(rospy.Duration(0.50), lambda x: self.updateCurrentPosition(self.position_updater))

    # Timer Callback: Update UI with correspond current position
    def updateCurrentPosition(self, timer_obj: rospy.Timer):
        with self.lock_position:
            x_pixel, y_pixel = self.gpsToScreenXY(self.current_latitude, self.current_longitude)
            if hasattr(self, 'lb_rover_icon') and self.lb_rover_icon:
                self.lb_rover_icon.move(x_pixel, y_pixel)
                # Ensure the rover icon is visible on the map.
                self.lb_rover_icon.show()
            else:
                print("Rover icon is not initialized.")
    
    # Subscriber Callback: Update current position members with GPS data
    def cbGPSData(self, data: gps):
        with self.lock_position:
            self.current_latitude = data.latitude
            self.current_longitude = data.longitude
            self.heading = data.heading_track
            self.height = data.height

    #https://stackoverflow.com/questions/16266809/convert-from-latitude-longitude-to-x-y
    def gpsToGlobalXY(self, latitude, longitude):
        # Calculates x based on cos of average of the latitudes
        x = self.earth_radius * longitude * math.cos((self.min_coord_lat + self.max_coord_lat)/2)
        # Calculates y based on latitude
        y = self.earth_radius * latitude

        return x, y
    
    
    def gpsToScreenXY(self, latitude, longitude):
        posX, posY = self.gpsToGlobalXY(latitude, longitude)
        
        perX = (posX - self.min_coord_x) / (self.max_coord_x - self.min_coord_x)
        perY = (posY - self.min_coord_y) / (self.max_coord_y - self.min_coord_y)

        # Calculate the screen coordinates
        x = self.min_coord_x + (self.max_coord_x - self.min_coord_x) * perX
        y = self.min_coord_y + (self.max_coord_y - self.min_coord_y) * perY

        # Return the screen coordinates
        return x, y
    
    
    def closePopUp(self):
        self.hide()
        # Consider not using `del self` unless absolutely necessary


