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

    ros::spin();


    // test
    // my_UI.MsgSpeakOut("Hey, this is Jack. I will pick the chocolate for you");
    // std::cout<<123<<std::endl;

    

    exit:
    	printf("按任意键退出 ...\n");
    	getchar();
    	MSPLogout(); //退出登录
    return 0;
}