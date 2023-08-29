#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from serial import Serial

def main():
    rospy.init_node('arduino_communication_node', anonymous=True)
    pub = rospy.Publisher('ros_to_arduino', String, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    arduino = Serial('/dev/ttyACM0', 9600)  # Update port as needed

    while not rospy.is_shutdown():
        message = "Hello Arduino!"
        arduino.write(message.encode())

        confirmation = arduino.readline().decode().strip()
        rospy.loginfo(f"Received confirmation from Arduino: {confirmation}")

        # Publish confirmation to a ROS topic
        pub.publish(confirmation)
        rate.sleep()

    arduino.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
