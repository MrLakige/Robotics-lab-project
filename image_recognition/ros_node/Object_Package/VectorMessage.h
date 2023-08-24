#ifndef __OBJECT_MESSAGE_VECTORMESSAGE_H__
#define __OBJECT_MESSAGE_VECTORMESSAGE_H__

#include <ros/ros.h>
#include <vector>
#include <stdint.h>
#include <iostream>

#include "object.h"

using namespace std;

namespace your_package_name{
    class VectorMessage{
    public:
        vector<Object> data;

        VectorMessage() {}

        explicit VectorMessage(const vector<Object>& data)
            : data(data)
        {}

        void deserialize(const ros::SerializedMessage& serialized_message){
            uint32_t size = serialized_message.getLength();
            data.resize(size);
            memcpy(data.data(), serialized_message.getData(), size);
        }

        uint32_t serializationLength() const{
            return data.size() * sizeof(Object);
        }

        void serialize(ros::SerializedMessage& serialized_message) const{
            serialized_message.resize(serializationLength());
            memcpy(serialized_message.getData(), data.data(), serialized_message.getLength());
        }
    };
}

#endif
