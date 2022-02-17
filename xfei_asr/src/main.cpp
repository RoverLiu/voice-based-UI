/**
 * @file main.cpp
 * @author Rover
 * @brief main file for the voice UI
 * @version 0.1
 * @date 2022-02-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "UI.h"
#include <stdlib.h>
#include <stdio.h>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char* argv[]) {
    // Initialize the ros
    ros::init( argc, argv, "UI");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv( "~" );

    UI my_UI(nh, nh_priv);

    my_UI.run_UI();    

    my_UI.Stop();
}