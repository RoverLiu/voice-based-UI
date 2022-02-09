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
#include "asr_record/asr_offline_record_sample.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "iat_publish_speak.h"
#include "tts_subscribe_speak.h"
#include "asr_record/play_audio.h"

#include "collabriative_bot_msgs/voice_command.h"

// int16_t g_order = ORDER_NONE;
// BOOL g_is_order_publiced = FALSE;
UserData asr_data;

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
    chocolate_command_pub = nh.advertise<collabriative_bot_msgs::voice_command>("chocolate2pick", 1000);

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

    // get current address
    char current_absolute_path[MAX_SIZE];
    //获取当前程序绝对路径
    int cnt = readlink("/proc/self/exe", current_absolute_path, MAX_SIZE);
    getcwd(current_absolute_path, MAX_SIZE);
    printf("current absolute path:%s\n", current_absolute_path);

    //init iflytek
    int ret = 0 ;
    /* 用户登录 */
	ret = MSPLogin(NULL, NULL, login_params);//第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://open.voicecloud.cn注册获取
	if (MSP_SUCCESS != ret)
	{
		printf("MSPLogin failed, error code: %d.\n", ret);
		/*goto exit ;*///登录失败，退出登录
        toExit();
	}

    memset(&asr_data, 0, sizeof(UserData));
    printf("构建离线识别语法网络...\n");
    ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
    if (MSP_SUCCESS != ret) {
        printf("构建语法调用失败!\n");
        Stop();
    }

    while (1 != asr_data.build_fini) {
        usleep(300 * 1000);
    }
    if (MSP_SUCCESS != asr_data.errcode) {
        Stop();
    }
    
    // message for user - init succeed
    printf("\n###########################################################################\n");
	printf("## 语音合成（Text To Speech，TTS） start \n");
	printf("## 语音识别 start \n");
    printf("## 离线识别语法网络构建完成，开始识别...\n");
    printf("## UI Ready\n");
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
    
    MsgSpeakOut(text, 0);
}

/**
 * @brief Regenerate the voice by the given text message
 * 
 * @param text 
 * @param state 0: generate message online, 1: message already exist, broadcast it
 */
void UI::MsgSpeakOut(const char* text, int state) {
    char cmd[2000];
    int         ret                  = MSP_SUCCESS;
    const char* session_begin_params = "voice_name = xiaoyan, text_encoding = utf8, sample_rate = 16000, speed = 50, volume = 50, pitch = 50, rdn = 2";
    const char* filename             = "tts_sample.wav"; //合成的语音文件名称

    printf("\n###########################################################################\n");
    if (state == 0) {
        /* 文本合成 */
        printf("Generate msg ...\n");
        ret = text_to_speech(text, filename, session_begin_params);
        if (MSP_SUCCESS != ret)
        {
            printf("text_to_speech failed, error code: %d.\n", ret);
        }
        printf("Msg Broadcasting\n");
    

        unlink("/tmp/cmd");  
        mkfifo("/tmp/cmd", 0777);  
        popen("mplayer -quiet -slave -input file=/tmp/cmd 'tts_sample.wav'","r");
        sleep(30);
    } else {
        std::cout<<"enter"<<std::endl;
        std::string absolute_address = "/home/rover/collabrative_robot_ws/src/summer_research/voice-based-UI/xfei_asr/audios/";
        std::string combined = absolute_address + text;
        
        int n = combined.length();
 
        // declaring character array
        char char_array[n + 1];
    
        // copying the contents of the
        // string to char array
        strcpy(char_array, combined.c_str());

        play_wav(char_array);
    }

    

    printf("Mplayer Run Success\n");
    printf("\n###########################################################################\n");
}

/**
 * @brief convert speech to text
 * 
 * @param msg No use, any message will wake up
 */
void UI::ConvertSpeechToTextCallback(const std_msgs::String::ConstPtr& msg) {

    std_msgs::String result;
    
    WakeUp();

    result = SpeechToTextConvention();

    word_heard_pub.publish(result);

}

/**
 * @brief log out and end service
 * 
 */
void UI::Stop() {
    MSPLogout();
    printf("请按任意键退出...\n");
    getchar();
    exit(0);
}

/**
 * @brief key method for UI (modify here for ur own)
 * 
 */
void UI::run_UI() {
    while (true)
    {
        run_ivw(NULL, ssb_param);
        if(g_is_awaken_succeed){
            printf("wake up\n");
            Ask_and_Response();
        }

    }
    
}

/**
 * @brief Define how UI response with different questions
 * 
 */
void UI::Ask_and_Response() {
    MsgSpeakOut("welcome.wav", 1);
    std::cout<< "Hi, this is Tom. I am here to serve you Chocolate. What brand of Chcolate do you prefer?" <<std::endl;

    int count = 0;

    while (count < max_wakeup)
    {
        // take user speech
        std_msgs::String result;
        char chocolate_res;
        
        // // check the sentence
        // WakeUp();

        // result = SpeechToTextConvention();

        // std::cout<<result.data<<std::endl;

        // chocolate_res = check_keywords(result.data);

        // check existence of keywords
        chocolate_res = check_chocolate();


        // publish command

        collabriative_bot_msgs::voice_command my_command;
        my_command.number_required = 1;

        if (chocolate_res == kitkat_chuncky || chocolate_res == kitkat_gold || chocolate_res == kitkat_mint) {
            std::cout<< "I heard you want a kitkat. I will pick it for you!" <<std::endl;
            MsgSpeakOut("kitkat.wav", 1);
            // publish command
            my_command.chocolate_type = chocolate_res;

            chocolate_command_pub.publish(my_command);



            break;
        } else if (chocolate_res == snickers)
        {
            std::cout<< "I heard you want a snickers. I will pick it for you!" <<std::endl;
            MsgSpeakOut("snickers.wav", 1);
            // publish command
            my_command.chocolate_type = 18;
            chocolate_command_pub.publish(my_command);







            break;
        } else {
            MsgSpeakOut("sorry.wav", 1);
            std::cout<<"Sorry, I have missed what you said. Could you please repeat that again?"<<std::endl;

        }
        

        count += 1;
    }
    

    
    
    
    return;
}

/**
 * @brief check whether there is a keyword in the sentence
 * 
 * @param keyworkds sentence to check
 * @return the type of chocolate 
 * 0: no choclate given
 * 1: brand kitkat
 * 2: brand snickers
 */
char UI::check_keywords(std::string sentence) {
    // std::string str = std::string(sentence);

    // check each brand with keyword defined in .h file
    for (std::unordered_map<char, std::list<std::string>>::iterator i = _chocolate_map.begin(); i != _chocolate_map.end(); i++) {
        std::cout << i->first << std::endl;
        std::list<std::string> temp = i->second;

        for (auto j = temp.begin(); j != temp.end(); ++j) {
            std::cout << *j << std::endl;

            size_t found = sentence.find(*j);
            if (found != std::string::npos) {
                return i->first;    
            }
        }
    }
    return 0;
}

char UI::check_chocolate() {
    ret = run_asr(&asr_data);
	if (MSP_SUCCESS != ret) {
		printf("离线语法识别出错: %d \n", ret);
		Stop();
	}
    std::cout << "result from cpp " << order_result << std::endl;
    return order_result;
    
}