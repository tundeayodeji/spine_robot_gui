import rclpy
from rclpy.node import Node
import serial

from std_msgs.msg import String


# Creates Serial Node
class SerialNode(Node):
    def __init__(self, ui):
        super().__init__('serial_node')
        self.ui = ui

        # Connects to the serial port used by the Arduino
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout = 1.0)

        # Subscribes to the topic that gui_node publishes on to receive and send messages to the Arduino
        self.ard_subscription = self.create_subscription(String, 'gui2ard', self.arduino_subscriber_callback, 10)
        
        # Timer callback that executes every 0.1 seconds
        self.timer = self.create_timer(0.1, self.timer_callback)


    # Sends message received from gui_node to Arduino via serial
    def arduino_subscriber_callback(self, msg):
        cmd = msg.data
        self.serial_port.write(cmd.encode())
        self.get_logger().info(f"Receiving: {cmd}")


    # Reads and displays any available serial data sent from Arduino
    # Resets GUI to initial state based on "false" keyword
    def timer_callback(self):
        info = self.serial_port.read(self.serial_port.in_waiting)
        decoded_info = info.decode(errors='ignore').strip()
        if decoded_info:
            self.get_logger().info(f"From Arduino: {decoded_info}")

        if "false" in decoded_info:
            self.task_running = False
            self.ui.selectFunctionComboBox.setEnabled(True)

            self.ui.p2_TopClampBtnTighten.setEnabled(True)
            self.ui.p2_TopClampBtnRelease.setEnabled(True)
            self.ui.p2_BotClampBtnTighten.setEnabled(True)
            self.ui.p2_BotClampBtnRelease.setEnabled(True)
            self.ui.p2_ScrewBtnContract.setEnabled(True)
            self.ui.p2_ScrewBtnExtend.setEnabled(True)
            self.ui.p2_BtnReleaseAllClamps.setEnabled(True)
            self.ui.p2_LblTaskRunning.hide()
            self.ui.p2_StopWgtBtn.setEnabled(False)

            self.ui.p3_CyclesSpinBox.setEnabled(True)
            self.ui.p3_SpeedSldr.setEnabled(True)
            self.ui.p3_BtnTaskRun.setEnabled(True)
            self.ui.p3_LblTaskRunning.hide()
            self.ui.p3_StopWgtBtn.setEnabled(False)

            self.ui.p4_CyclesSpinBox.setEnabled(True)
            self.ui.p4_SpeedSldr.setEnabled(True)
            self.ui.p4_BtnTaskRun.setEnabled(True)
            self.ui.p4_LblTaskRunning.hide()
            self.ui.p4_StopWgtBtn.setEnabled(False)