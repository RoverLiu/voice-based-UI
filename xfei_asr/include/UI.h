/**
 * @file UI.h
 * @author Dechuan Liu 
 * @brief This file provides a model for voice based user interface
 * Ideally, the UI should be able to take the user command from microphone and reply with speaker.
 * @version 0.1
 * @date 2022-01-21
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __UI_H
#define __UI_H

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include <iconv.h>

#include "ros/ros.h"
#include "std_msgs/String.h"

//----------------------------------------UI--------------------------------------
class UI 
{
    public:
        // constructor and deconstructor
        UI(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~UI();

        // methods
        void run_UI();      //run the UI
        




    private:
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;

        // method
        // take message generated from voice
        void MsgCallback(const std_msgs::String::ConstPtr& msg);

        // convert the message to speech
        void MsgSpeakOut(std::string msg);

        // wait for keyword
        bool KeywordReceived();
        
};


#endif
