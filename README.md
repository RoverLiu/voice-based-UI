# voice-based-UI
This package is a user interface for ROS developed based on 科大讯飞 https://global.xfyun.cn/

The package supports voice recognition, voice regeneration, and keyword wakeup.

Test version:
ROS-melodic
Ubuntu 18.04 LTS (64 bits)

# build
1. Change all appid to your own ID
2. Change all absolute address (/home/your_name/...)
3. replace all resource files in lib, libs, and res 
    (could be found in sdf packages from xfei - res is located in /bin/msc)
4. catkin_build

## common error
error code 25000 : Library did not load correctly

Try:    export LD_LIBRARY_PATH=/home/rover/voice_test_ws/src/voice-based-UI/xfei_asr/libs/x64/  (modify address)

