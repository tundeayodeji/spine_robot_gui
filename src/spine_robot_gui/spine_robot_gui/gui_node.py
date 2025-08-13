import rclpy
from rclpy.node import Node
from functools import partial

from std_msgs.msg import String


# Creates GUI Node
class GuiNode(Node):
    def __init__(self, ui):
        super().__init__('gui_node')
        self.ui = ui

        # Declares that gui_node publishes messages to serial_node (over gui2ard topic)
        self.gui_publisher = self.create_publisher(String, 'gui2ard', 10)

        # Button clicks in the GUI trigger callback execution
        self.ui.p2_TopClampBtnTighten.clicked.connect(partial(self.setup_callback, 1))
        self.ui.p2_TopClampBtnRelease.clicked.connect(partial(self.setup_callback, 2))
        self.ui.p2_BotClampBtnTighten.clicked.connect(partial(self.setup_callback, 3))
        self.ui.p2_BotClampBtnRelease.clicked.connect(partial(self.setup_callback, 4))
        self.ui.p2_ScrewBtnContract.clicked.connect(partial(self.setup_callback, 5))
        self.ui.p2_ScrewBtnExtend.clicked.connect(partial(self.setup_callback, 6))
        self.ui.p2_BtnReleaseAllClamps.clicked.connect(partial(self.setup_callback, 7))

        self.ui.p3_BtnTaskRun.clicked.connect(partial(self.drive_callback, 8))
        self.ui.p4_BtnTaskRun.clicked.connect(partial(self.drive_callback, 9))

        self.ui.p2_StopWgtBtn.clicked.connect(self.stop_pub_callback)
        self.ui.p3_StopWgtBtn.clicked.connect(self.stop_pub_callback)
        self.ui.p4_StopWgtBtn.clicked.connect(self.stop_pub_callback)

        # Links "cycles" value in "Forward" and "Backward Drive" tabs to callback that changes needle distance indicator
        self.needle_distance_per_cycle = 2
        self.ui.p3_CyclesSpinBox.valueChanged.connect(self.needle_distance)
        self.ui.p4_CyclesSpinBox.valueChanged.connect(self.needle_distance)

        # Hides "Task Running..." text initially
        self.ui.p2_LblTaskRunning.hide()
        self.ui.p4_LblTaskRunning.hide()
        self.ui.p4_LblTaskRunning.hide()


    # Callback for functionality of buttons in "Setup" GUI tab
    def setup_callback(self, case):
        # Creates and publishes message to send to Arduino
        msg = String()
        msg.data = f"{case}\n"
        self.gui_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        # Disables most visible GUI buttons, enables "Stop" button, shows "Task Running..." text
        self.ui.selectFunctionComboBox.setEnabled(False)
        self.ui.p2_TopClampBtnTighten.setEnabled(False)
        self.ui.p2_TopClampBtnRelease.setEnabled(False)
        self.ui.p2_BotClampBtnTighten.setEnabled(False)
        self.ui.p2_BotClampBtnRelease.setEnabled(False)
        self.ui.p2_ScrewBtnContract.setEnabled(False)
        self.ui.p2_ScrewBtnExtend.setEnabled(False)
        self.ui.p2_BtnReleaseAllClamps.setEnabled(False)
        self.ui.p2_LblTaskRunning.show()
        self.ui.p2_StopWgtBtn.setEnabled(True)


    # Callback for functionality in "Forward" and "Backward Drive" GUI tabs
    def drive_callback(self, case):
        # Sets values for cycles and speed to send to Arduino
        if case == 8:
            cycles = int(self.ui.p3_CyclesSpinBox.value())
            speed = int(self.ui.p3_SpeedSldr.value())
        
        elif case == 9:
            cycles = int(self.ui.p4_CyclesSpinBox.value())
            speed = int(self.ui.p4_SpeedSldr.value())
        
        
        # Creates and publishes message to send to Arduino
        msg = String()
        msg.data = f"{case},{cycles},{speed}\n"
        self.gui_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        
        # Disables most visible GUI buttons, enables "Stop" button, shows "Task Running..." text
        self.ui.selectFunctionComboBox.setEnabled(False)
        if case == 8:
            self.ui.p3_CyclesSpinBox.setEnabled(False)
            self.ui.p3_SpeedSldr.setEnabled(False)
            self.ui.p3_BtnTaskRun.setEnabled(False)
            self.ui.p3_LblTaskRunning.show()
            self.ui.p3_StopWgtBtn.setEnabled(True)
        
        elif case == 9:
            self.ui.p4_CyclesSpinBox.setEnabled(False)
            self.ui.p4_SpeedSldr.setEnabled(False)
            self.ui.p4_BtnTaskRun.setEnabled(False)
            self.ui.p4_LblTaskRunning.show()
            self.ui.p4_StopWgtBtn.setEnabled(True)


    # Callback for "Stop" button functionality
    def stop_pub_callback(self):
        # Creates and publishes message to send to Arduino
        msg = String()
        msg.data = "S"
        self.gui_publisher.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")

        # Disables "Stop" button, hides "Task Running..." text
        self.ui.p2_StopWgtBtn.setEnabled(False)
        self.ui.p3_StopWgtBtn.setEnabled(False)
        self.ui.p4_StopWgtBtn.setEnabled(False)
        self.ui.p2_LblTaskRunning.hide()
        self.ui.p4_LblTaskRunning.hide()
        self.ui.p4_LblTaskRunning.hide()
    

    # Callback to update displayed needle distance based on entered number of cycles
    def needle_distance(self):
        self.ui.p3_CyclesLblDistance.setText(f"{int(self.ui.p3_CyclesSpinBox.value()) * self.needle_distance_per_cycle}")
        self.ui.p4_CyclesLblDistance.setText(f"{int(self.ui.p4_CyclesSpinBox.value()) * self.needle_distance_per_cycle}")