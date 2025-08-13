import sys
import rclpy
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from PyQt5 import QtWidgets

from .gui import Ui_MainWindow
from .gui_node import GuiNode
from .serial_node import SerialNode


def main(args=None):
    rclpy.init(args=args)
    application = QtWidgets.QApplication(sys.argv)
    main_window = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(main_window)

    gui_node = GuiNode(ui)
    serial_node = SerialNode(ui)

    executor = MultiThreadedExecutor()
    executor.add_node(gui_node)
    executor.add_node(serial_node)

    thread = Thread(target=executor.spin)
    thread.start()

    try:
        main_window.show()
        sys.exit(application.exec_())

    finally:
        gui_node.destroy_node()
        serial_node.destroy_node()
        executor.shutdown()


if __name__ == '__main__':
    main()