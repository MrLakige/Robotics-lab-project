#include "ros/ros.h"
#include "std_msgs/String.h"

#include "lib/image_detector.h"
#include <iostream>
#include <vector>

#include "json/json.h" 
#include "Object_Package/VectorMessage.h"


using namespace std;

static image_detector detector;

void messageCallback(const std_msgs::String::ConstPtr& msg) {
    string message = msg->data;
    string keyword = "image";
    string keyword1 = "image to string";
    string keyword2 = "image to json";

    // with vector
    if (message.find(keyword) != string::npos) {
        vector<Object> responseVet = detector.image_recognition();
        // Create a response message
        ros::NodeHandle node;
        ros::Publisher pub = node.advertise<Object_Package::VectorMessage>("vector_topic", 10);
        Object_Package::VectorMessage msg;
        response_msg.data = responseVet;
        // Publish the response message
        pub.publish(response_msg);
    }

    // with string
    if (message.find(keyword1) != string::npos) {
        vector<Object> responseVet = detector.image_recognition();
        string response;
        for (const Object &obj : responseVet) {
            response<<obj;
        }
        // Create a response message
        ros::NodeHandle node;
        ros::Publisher pub = node.advertise<std_msgs::String>("response_topic", 10);
        std_msgs::String response_msg;
        response_msg.data = response;
        // Publish the response message
        pub.publish(response_msg);
    }

    // with json
    if (message.find(keyword) != string::npos) {
        vector<Object> responseVet = detector.image_recognition();

        ros::NodeHandle node;
        ros::Publisher pub = node.advertise<std_msgs::String>("json_vector_topic", 10);
        
        // Convert vector of class objects to JSON string
        Json::Value json_data(Json::arrayValue);
        for (const auto& obj : responseVet) {
            Json::Value json_object;
            json_object["name"] = obj.getName();
            json_object["x"] = obj.getX();
            json_object["y"] = obj.getY();
            json_object["z"] = obj.getZ();

            json_data.append(json_object);
        }

        // Convert JSON object to string
        Json::StreamWriterBuilder writer;
        std::string json_string = Json::writeString(writer, json_data);

        // Create a ROS message
        std_msgs::String msg;
        msg.data = json_string;

        pub.publish(msg);
    }

}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processor");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("message_topic", 10, messageCallback);

    ros::spin();

    return 0;
}
