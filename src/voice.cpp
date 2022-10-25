/**
* @file voice.cpp
* @brief machine voice
* @author Michikuni Eguchi
* @date 2022.10.25
* @details 機械音声を流す
*
*/
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32.h>
#include <string>
#include "plat/robot_status.h"

class voice
{
private:
    ros::NodeHandle nh;

    //subscriber
    ros::Subscriber wpMode_sub;
    ros::Subscriber mode_sub;
    ros::Subscriber wpSet_sub;
    //publisher
    ros::Publisher voiceTxt_pub;

    //function
    std_msgs::UInt8MultiArray mode_array;
    void wpMode_callback(const std_msgs::UInt8MultiArray& modeArray_message)
    {
        if(mode_array != modeArray_message){
            isState_updata = true;
        }

        mode_array = modeArray_message;
    }

    std_msgs::String mode_in;
    void mode_callback(const std_msgs::String& mode_message)
    {
        if(mode_in != mode_message){
            isState_updata = true;
        }
        mode_in = mode_message;
    }

    std_msgs::Int32 targetWp;
    void wayPoint_set_callback(const std_msgs::Int32 wpset_message)
    {
        targetWp = wpset_message;
    }

    //variables
    double rate;
    bool isState_updata = false;

public:
    voice();
    void updata();
};

voice::voice()
{
    ros::NodeHandle pnh("~");
    pnh.param<double>("loop_rate", rate, 5);

    //subscriber
    wpMode_sub = nh.subscribe("wayPoint/mode", 10, &voice::wpMode_callback, this);
    mode_sub = nh.subscribe("mode", 10 , &voice::mode_callback, this);
    wpSet_sub = nh.subscribe("wayPoint/set", 10, &voice::wayPoint_set_callback, this);
    //publisher
    voiceTxt_pub = nh.advertise<std_msgs::String>("plat/voice_txt", 10);
}

void voice::updata()
{
    static ros::Rate loop_rate(rate);

    if(mode_array.data.size()>0){
        std_msgs::String voice_txt;
        if(mode_array.data[targetWp.data] == (uint8_t)robot_status::stop){
            voice_txt.data = "まもなく停止します";
        }

        if(mode_in.data == robot_status_str(robot_status::stop)){
            voice_txt.data = "停止中";
        }

        if(isState_updata){
            voiceTxt_pub.publish(voice_txt);
            isState_updata = false;
        }
    }

    loop_rate.sleep();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "voice");

    voice v;
    while(ros::ok())
    {
        v.updata();
        ros::spinOnce();
    }

    return 0;
}