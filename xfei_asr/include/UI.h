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
#include <errno.h>
#include <iconv.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <unordered_map>
 #include <list>

#include "qtts.h"
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"
#include "UI.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iat_publish_speak.h"
#include "tts_subscribe_speak.h"

// #include "asr_record/asr_record.h"
#include "awaken/awaken.h"
#include "asr_record/play_audio.h"

#define MAX_SIZE 100


//----------------------------------------UI--------------------------------------
class UI 
{
    public:
        // constructor and deconstructor
        UI(ros::NodeHandle nh, ros::NodeHandle nh_priv);
        ~UI();

        // methods
        void run_UI();      //run the UI

        void MsgSpeakOut(const char* text, int state);

        
        void Stop();        // stop and log out

    private:
        // data
        // ROS NodeHandle
        ros::NodeHandle _nh;
        ros::NodeHandle _nh_priv;
        ros::Subscriber wakeup_sub;
        ros::Subscriber word_to_say_sub;

        ros::Publisher word_to_say_pub;
        ros::Publisher word_heard_pub;

        
        int         ret                  = MSP_SUCCESS;

        // variables below might be changed based on the condition
        const char* login_params         = "appid = 1638a95e, work_dir = ., engine_start = ivw, ivw_res_path =fo|/home/rover/voice_test_ws/src/voice-based-UI/xfei_asr/res/ivw/wakeupresource.jet";//登录参数,appid与msc库绑定,请勿随意改动
        const char *ssb_param            = "sst=wakeup, ivw_threshold=0:1450,ivw_res_path =fo|/home/rover/voice_test_ws/src/voice-based-UI/xfei_asr/res/ivw/wakeupresource.jet";
        
        // maximum wake up time
        const int max_wakeup = 3 ;
        // // all potential names for snickers
        // enum snickers {
        //     snickers
        // };

        // // all potential names for kitkat
        // enum kitkat {
        //     kitkat
        // };

        std::unordered_map<char, std::list<std::string>> _chocolate_map = {
            {1,{"kitkat","kidkat","kitkad", "take care"}}, // potential kitkat
            {2,{"snickers","snikers","snicker", "speaker", "sneaker", "snake", "brown"}}  // potential snickers
        };

        
        /*
        * rdn:           合成音频数字发音方式
        * volume:        合成音频的音量
        * pitch:         合成音频的音调
        * speed:         合成音频对应的语速
        * voice_name:    合成发音人
        * sample_rate:   合成音频采样率
        * text_encoding: 合成文本编码格式
        *
        * 详细参数说明请参阅《讯飞语音云MSC--API文档》
        */


        // method
        void init();

        // // take message generated from voice
        // void MsgCallback(const std_msgs::String::ConstPtr& msg);

        // take message and convert to voice
        void VoiceRegenerationCallback(const std_msgs::String::ConstPtr& msg);

        // wait for keyword
        bool KeywordReceived();
        
        // convert the message to speech
        // void MsgSpeakOut(const char* text, int state);

        // convert speech to text
        void ConvertSpeechToTextCallback(const std_msgs::String::ConstPtr& msg);

        // ask user for chocolate brand and provide the service 
        void Ask_and_Response();

        // check whether there is a key word in the sentence
        char check_keywords(std::string sentence);

        // identify the audio and check the chocolate
        char check_chocolate();

};


#endif
