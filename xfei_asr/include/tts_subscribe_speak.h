#ifndef __TTS_SUBSCRIBE_SPEAK_H
#define __TTS_SUBSCRIBE_SPEAK_H
#include "ros/ros.h"
#include "std_msgs/String.h"

int text_to_speech(const char* src_text, const char* des_path, const char* params);

#endif