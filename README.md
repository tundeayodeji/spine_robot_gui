# spine_robot_gui
GUI to facilitate control of needle insertion robot for in-development semi-autonomous vertebroplasty system

---

1. First, open the Arduino IDE and upload the "**gui_controlled_spine_robot.ino**" code from the "**Arduino Code**" directory in the inner spine_robot_gui package directory (spine_robot_gui/src/spine_robot_gui/Arduino Code) to the Arduino

2. While still in the Arduino IDE, check the Arduino board's port and make sure it matches "**/dev/ttyACM0**"; if it's something different, see the Troubleshooting steps below

3. Clone this repository to set up the spine_robot_gui workspace by running the following command in a new terminal: `git clone https://github.com/tundeayodeji/spine_robot_gui.git`

4. Move into the "spine_robot_gui" workspace directory (`cd spine_robot_gui`) and enter the following command to check for and install any necessary dependencies: `rosdep install --from-paths src -y --ignore-src`

5. Build the GUI package by running `colcon build` in the terminal

6. Open a new terminal, move into the "spine_robot_gui" workspace directory (`cd spine_robot_gui`), and source the workspace (`source install/setup.bash`)

7. Finally, run the GUI by entering the following command: `ros2 run spine_robot_gui main`

---
Troubleshooting:
---

**Make sure to rebuild the package, source the workspace again, and then run the GUI after editing and saving any package files to ensure changes are reflected!**

\
**If your board's port is not "/dev/ttyACM0"**:

Update **line 15** of "serial_node.py" (spine_robot_gui/src/spine_robot_gui/spine_robot_gui/serial_node.py) to reflect the correct port

\
**If you receive an error similar to: "Could not open port /dev/ttyACM0 . . . No such file or directory: '/dev/ttyACM0' . . ."**:

Make sure your Arduino board is connected and/or update "serial_node.py" as described above, if necessary

\
**If you receive an error similar to: "Could not open port /dev/ttyACM0 . . . Device or resource busy: '/dev/ttyACM0' . . ."**:

Make sure the Serial Monitor in the Arduino IDE is not open and that you're not otherwise communicating with/sending data to the Arduino
