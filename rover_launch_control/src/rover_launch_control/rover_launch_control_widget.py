from __future__ import (print_function, absolute_import, division, unicode_literals)

import os
import rospkg
import rospy
import roslaunch
import rosparam
import rostopic
import os
import time
from python_qt_binding import loadUi
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QLabel, QPushButton, QAbstractButton, QComboBox, QApplication
from threading import Lock
from copy import copy
from robotnik_msgs.msg import inputs_outputs
from robotnik_msgs.srv import set_digital_output
import rosservice

# Styling "MACROS"
STYLE_DEFAULT = ""
STYLE_SELECTED = "color: white; background-color: green"
STYLE_DISABLE = "color: white; background-color: grey"
STYLE_LIMITING = "color: black; background-color: yellow"
STYLE_WARN = "color: black; background-color: rgb(255, 124, 0);"

# Constant
RELAY_BOARD_SERVICE_NAME = "/rly_08_node/set_digital_outputs"
RELAY_BOARD_MESSAGE_NAME = "/rly_08_node/status"

# Global Variables
launchmode_local = 'false'

class LaunchInterface():
    def __init__(self, uuid, pkg_name: str, launchfile_name: str, parameters: list = []):
        self.name_pkg = pkg_name
        self.name_launchfile = launchfile_name
        self.parameters = parameters
        self.uuid = uuid
        self.launch_handler_started: bool = False
        self.launch_handler = None

    def update_param(self):
        self.launchfile = roslaunch.rlutil.resolve_launch_arguments([self.name_pkg, self.name_launchfile + '.launch'])
        if self.parameters == None:
            parameters = ['local_only:=' + launchmode_local]
        else:
            parameters = copy(self.parameters)
            parameters = ['local_only:=' + launchmode_local]

        self.launchfile = [(self.launchfile[0], parameters)]

class BandwidthInterface:
    def __init__(self, topic_name, window: float = 5.0):
        self.topic_name = topic_name
        self.window = window
        self.bw_api = rostopic.ROSTopicBandwidth(self.window);
        self.sub = rospy.Subscriber(self.topic_name, rospy.AnyMsg, self.bw_api.callback)
        self._flag: bool = False
        self.bw = None
        self.t0 = None
        self.tn = None

    def get_bw(self) -> int :
        """print the average publishing rate to screen"""
        if len(self.bw_api.times) < 2:
            return
        with self.bw_api.lock:
            n = len(self.bw_api.times)
            tn = time.time()
            t0 = self.bw_api.times[0]
            
            total = sum(self.bw_api.sizes)
            bytes_per_s = total / (tn - t0)
            mean = total / n

            # min and max
            max_s = max(self.bw_api.sizes)
            min_s = min(self.bw_api.sizes)

        #min/max and even mean are likely to be much smaller, but for now I prefer unit consistency
        if bytes_per_s < 1000:
            bw, mean, min_s, max_s = ["%.2fB"%v for v in [bytes_per_s, mean, min_s, max_s]]
        elif bytes_per_s < 1000000:
            bw, mean, min_s, max_s = ["%.2fKB"%(v/1000) for v in [bytes_per_s, mean, min_s, max_s]]  
        else:
            bw, mean, min_s, max_s = ["%.2fMB"%(v/1000000) for v in [bytes_per_s, mean, min_s, max_s]]
        
        self.bw = bw
        self.t0 = t0
        self.tn = tn
        return

class RoverLaunchControlWidget(QtWidgets.QWidget):
    
    def __init__(self):
        super(RoverLaunchControlWidget, self).__init__()

        ui_file = os.path.join(rospkg.RosPack().get_path('rover_launch_control'), 'resource', 'rover_launch_control.ui')
        loadUi(ui_file, self)
        self.setObjectName('RoverLaunchControlWidget')

        # I don't really understands qt and ros python threading so putting some locks on all button callback to only 
        # execute one at a time (probably useless but more secure nonetheless)
        self.lockLaunchFile = Lock()
        self.lockLaunchCameraStream = Lock()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        self.resolutionDict = {
            "1080p": ("1920", "1080"),
            "720p": ("1080", "720"),
            "480p": ("640", "480"),
            "360p": ("480", "360"),
            "144p": ("192", "144"),
            }

        #Mode selection buttons
        #setting default value
        global launchmode_local
        launchmode_local = 'true' 

        if launchmode_local == 'true':
            self.pb_local_true.setStyleSheet(STYLE_SELECTED)
            self.pb_local_false.setStyleSheet(STYLE_WARN)
        elif launchmode_local == 'false':
            self.pb_local_false.setStyleSheet(STYLE_SELECTED)
            self.pb_local_false.setStyleSheet(STYLE_WARN)
        else:
            exit()
            
        self.pb_local_true.released.connect(lambda: self.localModeSelection(True, self.pb_local_true, [self.pb_local_false]))
        self.pb_local_false.released.connect(lambda: self.localModeSelection(False, self.pb_local_false, [self.pb_local_true]))

        #rover_control
        base_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='base')
        self.pb_base.released.connect(lambda: self.launchFile(base_interface, self.pb_base))

        rover_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='rover')
        self.pb_rover.released.connect(lambda: self.launchFile(rover_interface, self.pb_rover))

        can_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='can')
        self.pb_can.released.connect(lambda: self.launchFile(can_interface, self.pb_can))

        camera_control_server_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='camera_control_server')
        self.pb_panorama_server.released.connect(lambda: self.launchFile(camera_control_server_interface, self.pb_panorama_server))

        #rover_arm
        joints_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='joints')
        self.pb_joints.released.connect(lambda: self.launchFile(joints_interface, self.pb_joints))

        controller_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_arm', launchfile_name='controller')
        self.pb_controller.released.connect(lambda: self.launchFile(controller_interface, self.pb_controller))

        #cameras
        cam_arducam_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_arducam')
        self.pb_cam_arducam_launch.released.connect(lambda: self.launchCameraStream("cam_arducam",
                                                                                     120,
                                                                                     cam_arducam_interface,
                                                                                     self.pb_cam_arducam_launch,
                                                                                     self.cb_cam_arducam_resolution,
                                                                                     self.pb_current_cam_arducam_resolution,
                                                                                     self.cb_cam_arducam_framerate,
                                                                                     self.pb_current_cam_arducam_framerate))
        self.pb_cam_arducam_apply.released.connect(lambda: self.ApplyCameraStream("cam_arducam",
                                                                                   120,
                                                                                   cam_arducam_interface,
                                                                                   self.pb_cam_arducam_launch,
                                                                                   self.cb_cam_arducam_resolution,
                                                                                   self.pb_current_cam_arducam_resolution,
                                                                                   self.cb_cam_arducam_framerate,
                                                                                   self.pb_current_cam_arducam_framerate))

        cam_sonix_interface = LaunchInterface(uuid= uuid, pkg_name= 'rover_control', launchfile_name='cam_sonix')
        self.pb_cam_sonix_launch.released.connect(lambda: self.launchCameraStream("cam_sonix",
                                                                                     30,
                                                                                     cam_sonix_interface,
                                                                                     self.pb_cam_sonix_launch,
                                                                                     self.cb_cam_sonix_resolution,
                                                                                     self.pb_current_cam_sonix_resolution,
                                                                                     self.cb_cam_sonix_framerate,
                                                                                     self.pb_current_cam_sonix_framerate))
        self.pb_cam_sonix_apply.released.connect(lambda: self.ApplyCameraStream("cam_sonix",
                                                                                   30,
                                                                                   cam_sonix_interface,
                                                                                   self.pb_cam_sonix_launch,
                                                                                   self.cb_cam_sonix_resolution,
                                                                                   self.pb_current_cam_sonix_resolution,
                                                                                   self.cb_cam_sonix_framerate,
                                                                                   self.pb_current_cam_sonix_framerate))


        #Periodic tasks
        cam_arducam_bandwidth = BandwidthInterface("/cam_arducam/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_arducam_bandwidth, self.pb_current_cam_arducam_bw));
    
        cam_sonix_bandwidth = BandwidthInterface("/cam_sonix/packet/compressed")
        self.bandwidth_updater = rospy.Timer(rospy.Duration(0.5), lambda x: self.bandwidthInfoUpdate(self.bandwidth_updater, cam_sonix_bandwidth, self.pb_current_cam_sonix_bw));

        #Lights
        self.pb_lights.released.connect(lambda: self.toggleLights(self.pb_lights, 1))
        self.light_status_updater = rospy.Timer(rospy.Duration(1.0), lambda x: self.updateLightStatus(self.light_status_updater, self.pb_lights, 1))

    def localModeSelection(self, value: bool, pb_toggle_self: QPushButton, pb_toggle_friends: "list[str]"):
        global launchmode_local
        launchmode_local = str(value)
        pb_toggle_self.setStyleSheet(STYLE_SELECTED)
        for friends in pb_toggle_friends:
            friends.setStyleSheet(STYLE_DEFAULT)
        rospy.loginfo("Already launched nodes needs to be restarted for mode to apply")

    def launchFile(self, launch_interface: LaunchInterface, button: QPushButton):
        with self.lockLaunchFile:
            QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
            try:
                if not launch_interface.launch_handler_started:
                    launch_interface.update_param()
                    launch_interface.launch_handler = roslaunch.parent.ROSLaunchParent(launch_interface.uuid, launch_interface.launchfile)
                    launch_interface.launch_handler_started = True
                    button.setStyleSheet(STYLE_SELECTED)
                    rospy.loginfo("Starting " + launch_interface.launchfile[0][0] + ' ' + ' '.join(launch_interface.launchfile[0][1]))
                    launch_interface.launch_handler.start()
                else:
                    launch_interface.launch_handler_started = False
                    button.setStyleSheet(STYLE_DEFAULT)
                    launch_interface.launch_handler.shutdown()

                    rospy.logwarn(launch_interface.name_pkg + ' ' + launch_interface.name_launchfile + '.launch shutdowned')
            except:
                button.setStyleSheet(STYLE_WARN);
            
            finally:
                QApplication.restoreOverrideCursor()

    def launchCameraStream(self,
                           cameraName: str,
                           cameraFramerate:int,
                           launch_interface:LaunchInterface,
                           button: QPushButton,
                           cb_resolution: QComboBox,
                           pb_current_resolution: QPushButton,
                           cb_framerate:QComboBox,
                           pb_current_framerate: QPushButton):
        with self.lockLaunchCameraStream:
            if not launch_interface.launch_handler_started:
                rosparam.set_param('/' + cameraName + '/hardware/framerate', str(cameraFramerate))
                framerate = int(cb_framerate.currentText())
                param_skip = int(cameraFramerate/framerate)-1 
                rosparam.set_param(cameraName + '/compressed_packet_controller/skip', str(param_skip))
                
                if ((param_skip + 1) * framerate != cameraFramerate):
                    rospy.logerr("Wrong framerate parameters node can't start")
                    return

                try:
                    resolution = cb_resolution.currentText()
                except KeyError:
                    rospy.logerr("Selected resolution isn't defined ")
                    return
                
                param_imageWidth = self.resolutionDict[resolution][0]
                param_imageHeight = self.resolutionDict[resolution][1]
                rosparam.set_param('/' + cameraName + '/hardware/image_width', param_imageWidth)
                rosparam.set_param('/' + cameraName + '/hardware/image_height', param_imageHeight)
                
                rospy.logwarn("Selected resolution: " + resolution + "(" + param_imageWidth + "x" + param_imageHeight + ")")
                rospy.logwarn("selected framerate: " + str(framerate))

                pb_current_framerate.setText("Fps: " + str(framerate))
                pb_current_resolution.setText("Res: " + resolution)
            else:
                pb_current_framerate.setText("")
                pb_current_resolution.setText("")


            self.launchFile(launch_interface, button)

    def ApplyCameraStream(self,
                cameraName: str,
                cameraFramerate:int,
                launch_interface:LaunchInterface,
                button: QPushButton,
                cb_resolution: QComboBox,
                pb_current_resolution: QPushButton,
                cb_framerate:QComboBox,
                pb_current_framerate: QPushButton):
    
        if launch_interface.launch_handler_started:
            for i in range(2):
                self.launchCameraStream(cameraName, cameraFramerate, launch_interface, button, cb_resolution, pb_current_resolution, cb_framerate, pb_current_framerate)
        else:
            rospy.logwarn(cameraName + " is not started yet")    

    def bandwidthInfoUpdate(self, timer_obj: rospy.Timer, bandwidth_interface: BandwidthInterface, pb_current_bw: QPushButton):
        bandwidth_interface.get_bw();
                                              # No news msgs in last 2 seconds
        if bandwidth_interface.bw == None or bandwidth_interface.tn == None or bandwidth_interface.t0 == None:
            if not bandwidth_interface._flag:
                pb_current_bw.setText("")
                rospy.logdebug("Topic: " + bandwidth_interface.topic_name + " not published yet")
                bandwidth_interface._flag = True
        elif (bandwidth_interface.tn-bandwidth_interface.t0) > bandwidth_interface.window:
            if not bandwidth_interface._flag:
                pb_current_bw.setText("")
                rospy.logdebug("Topic " + bandwidth_interface.topic_name + " stopped")
                bandwidth_interface._flag = True
        else:
            bandwidth_interface._flag = False
            pb_current_bw.setText(str(bandwidth_interface.bw) + "/s")

    def updateLightStatus(self, timer_obj: rospy.Timer, button: QPushButton, output_index: int):
        # Not locking way to check for service existing
        lst_services: list = rosservice.get_service_list()
        if RELAY_BOARD_SERVICE_NAME not in lst_services:
            button.setStyleSheet(STYLE_DISABLE)
        
        # Actual callback
        try:
            data: inputs_outputs = rospy.wait_for_message(RELAY_BOARD_MESSAGE_NAME, inputs_outputs, rospy.Duration(0.1))

            if data.digital_outputs[output_index]:
                button.setStyleSheet(STYLE_SELECTED)
            else:
                button.setStyleSheet(STYLE_DEFAULT)

        except:
            rospy.logwarn_throttle(10, "Error rover_launch_control can't get relay status")

    def toggleLights(self, button: QPushButton, index: int):
        QApplication.setOverrideCursor(QtCore.Qt.WaitCursor)
        try :
            rospy.wait_for_service(RELAY_BOARD_SERVICE_NAME, rospy.Duration(1.0))

            set_digital_output_service: set_digital_output = rospy.ServiceProxy(RELAY_BOARD_SERVICE_NAME, set_digital_output)
            
            data: inputs_outputs = rospy.wait_for_message(RELAY_BOARD_MESSAGE_NAME, inputs_outputs, rospy.Duration(1.0))
            if data.digital_outputs.digital_outputs[index] == True:
                set_digital_output_service(index, False)
            else:
                set_digital_output_service(index, True)

        except:
            rospy.logwarn("Error calling relay board service")
            button.setStyleSheet(STYLE_WARN)
        finally:
            QApplication.restoreOverrideCursor()
