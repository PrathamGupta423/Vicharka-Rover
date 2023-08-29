#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

String received_message = "";

void messageCallback(const std_msgs::String& msg){
  // This function is called when a message is received from ROS
  received_message = msg.data;
  nh.loginfo("Received message from ROS: " + received_message);
  nh.loginfo("Storing message in variable...");
}

ros::Subscriber<std_msgs::String> sub("ros_to_arduino", &messageCallback);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.loginfo("Arduino initialized.");

  Serial.begin(9600);
}

void loop() {
  nh.spinOnce();

  if (received_message != "") {
    // Send a confirmation message back to ROS
    nh.loginfo("Sending confirmation message to ROS...");
    nh.loginfo("Message stored: " + received_message);
    nh.loginfo("Sending confirmation...");
    nh.loginfo("Confirmation sent.");
    received_message = "";  // Reset the message
  }
}
