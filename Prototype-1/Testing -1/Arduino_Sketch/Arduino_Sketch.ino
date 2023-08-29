#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

String received_message = "";

void messageCallback(const std_msgs::String& msg){
    received_message = msg.data;
    nh.loginfo("Received message from ROS: " + received_message);

    // Send confirmation back to ROS
    nh.loginfo("Sending confirmation back to ROS...");
    nh.loginfo("Message stored: " + received_message);
    nh.loginfo("Sending confirmation...");
    nh.loginfo("Confirmation sent.");
    nh.loginfo("=========");
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
}