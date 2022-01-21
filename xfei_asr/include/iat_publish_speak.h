#ifndef __IAT_PUBLISH_SPEAK_H
#define __IAT_PUBLISH_SPEAK_H
#include "ros/ros.h"
#include "std_msgs/String.h"

void WakeUp();
void toExit();

static void demo_mic(const char* session_begin_params);

std_msgs::String SpeechToTextConvention();

#endif