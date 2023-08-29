# ~/catkin_ws/src/my_package_name/scripts/arduino_communication_node.py

#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from serial import Serial

def callback(data):
    # This function is called when a message is received from ROS
    arduino.write(data.data.encode())

    # Wait for confirmation message from Arduino
    confirmation = arduino.readline().decode().strip()
    rospy.loginfo(f"Received confirmation from Arduino: {confirmation}")

rospy.init_node('arduino_communication_node')
port = '/dev/ttyACM0'  # Adjust the port based on your Arduino connection
baud_rate = 9600

arduino = Serial(port, baud_rate)
rospy.Subscriber('ros_to_arduino', String, callback)

rospy.spin()

arduino.close()
