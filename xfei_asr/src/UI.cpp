/**
 * @file UI.cpp
 * @author Dechuan Liu
 * @brief Voice based UI
 * @version 0.1
 * @date 2022-01-21
 * 
 * @copyright Copyright (c) 2022;
 * 
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <iconv.h>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>

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

/**
 * @brief Construct a new UI::UI object
 * Set topics for ROS
 * Initalize
 * 
 * @param nh 
 * @param nh_priv 
 */
UI::UI(ros::NodeHandle nh, ros::NodeHandle nh_priv) :_nh(nh),_nh_priv(nh_priv){
    // set up ros
    ros::Rate loop_rate(10);

    wakeup_sub = nh.subscribe("xfwakeup", 1000, & UI::ConvertSpeechToTextCallback, this);
    word_to_say_sub = nh.subscribe("xfwords", 1000, & UI::VoiceRegenerationCallback, this);

    word_to_say_pub = nh.advertise<std_msgs::String>("xfwords", 1000);
    word_heard_pub = nh.advertise<std_msgs::String>("xfspeech", 1000);

    // init
    init();
    
}

/**
 * @brief Destroy the UI::UI object
 * 
 */
UI::~UI() {
    
}

/**
 * @brief Initialize and log in for xfei
 * 
 */
void UI::init() {
    /* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		/*goto exit ;*///登录失败，退出登录
        toExit();
	}

    printf("\n###########################################################################\n");
	printf("## 语音合成（Text To Speech，TTS） start \n");
	printf("###########################################################################\n\n");
}

/**
 * @brief callback for voice regeneration
 * Generate the voice when a new message is given in the topic "xfwords"
 * 
 * @param msg 
 */
void UI::VoiceRegenerationCallback(const std_msgs::String::ConstPtr& msg) {
    const char* text;
    
    std::cout<<"I heard :"<<msg->data.c_str()<<std::endl;
    text = msg->data.c_str(); 
    
    MsgSpeakOut(text);
}

/**
 * @brief Regenerate the voice by the given text message
 * 
 * @param text 
 */
void UI::MsgSpeakOut(const char* text) {
    char cmd[2000];
    int         ret                  = MSP_SUCCESS;
    const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    const char* filename             = "tts_sample.wav"; //合成的语音文件名称

    /* 文本合成 */
    printf("\n###########################################################################\n");
    printf("开始合成 ...\n");
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }
    printf("合成完毕\n");


    unlink("/tmp/cmd");  
    mkfifo("/tmp/cmd", 0777);  
    popen("mplayer -quiet -slave -input file=/tmp/cmd 'tts_sample.wav'","r");
    sleep(30);
    printf("Mplayer Run Success\n");
    printf("\n###########################################################################\n");

}

void UI::ConvertSpeechToTextCallback(const std_msgs::String::ConstPtr& msg) {

    std_msgs::String result;
    
    WakeUp();

    result = SpeechToTextConvention();

    word_heard_pub.publish(result);

}
