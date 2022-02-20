# voice-based-UI
This package is a user interface for ROS developed based on [科大讯飞](https://global.xfyun.cn/)

The package supports voice recognition, voice regeneration, and keyword wakeup.

Test version:
ROS-melodic
Ubuntu 18.04 LTS (64 bits)

# build
```
1. Apply an account from xfei and get your own APPID. Change all appid in the package to your own ID.
2. Change all absolute address (/home/your_name/...). The relative location did not work for me.
3. Download the sdk files from the website and replace all resource files in lib, libs, and res file
    (res file could be found in sdf packages from xfei - res is located in /bin/msc) 
4. catkin_build
```
One thing to notice, the bnf file defines the keywordes to identify. Modify that if you need.

## common error
error code 25000 : Library did not load correctly

Try:    export LD_LIBRARY_PATH=/home/rover/voice_test_ws/src/voice-based-UI/xfei_asr/libs/x64/  (modify address)

