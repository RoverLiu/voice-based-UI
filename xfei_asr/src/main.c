#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>

#include "../include/msc/msp_cmn.h"
#include "../include/msc/msp_errors.h"

#include "../include/asr_record/asr_record.h"
#include "../include/awaken/awaken.h"
#include "../include/asr_record/play_audio.h"


#define lgi_param_a "appid = 1638a95e,engine_start = ivw,work_dir = .,ivw_res_path =fo|"
#define lgi_param_b concat(lgi_param_a, PACKAGE_PATH)
const char *lgi_param = concat(lgi_param_b, "res/ivw/wakeupresource.jet"); //使用唤醒需要在此设置engine_start = ivw,ivw_res_path =fo|xxx/xx 启动唤醒引擎
const char *ssb_param = "sst=wakeup, ivw_threshold=0:1450,ivw_res_path =fo|/home/rover/voice_test_ws/src/voice-based-UI/xfei_asr/res/ivw/wakeupresource.jet";



int16_t g_order = ORDER_NONE;
BOOL g_is_order_publiced = FALSE;
UserData asr_data;


#define MAX_SIZE 100
int main(int argc, char **argv)
{
  char current_absolute_path[MAX_SIZE];
  //获取当前程序绝对路径
  int cnt = readlink("/proc/self/exe", current_absolute_path, MAX_SIZE);
  getcwd(current_absolute_path, MAX_SIZE);
  printf("current absolute path:%s\n", current_absolute_path);



  int ret = 0 ;

  ret = MSPLogin(NULL, NULL, lgi_param);
  if (MSP_SUCCESS != ret)
  {
    printf("MSPLogin failed, error code: %d.\n", ret);
    goto exit ;//登录失败，退出登录
  }


  memset(&asr_data, 0, sizeof(UserData));
  printf("构建离线识别语法网络...\n");
  ret = build_grammar(&asr_data);  //第一次使用某语法进行识别，需要先构建语法网络，获取语法ID，之后使用此语法进行识别，无需再次构建
  if (MSP_SUCCESS != ret) {
    printf("构建语法调用失败!\n");
    goto exit;
  }
  while (1 != asr_data.build_fini)
    usleep(300 * 1000);
  if (MSP_SUCCESS != asr_data.errcode)
    goto exit;
  printf("离线识别语法网络构建完成，开始识别...\n");


  while (1)
  {

    run_ivw(NULL, ssb_param); 
    printf("finish run_ivw\n");
    if(g_is_awaken_succeed){
      printf("begin to run asr\n");
      run_asr(&asr_data);
      g_is_awaken_succeed = FALSE;
    }
    printf("%d:%d\n", g_is_order_publiced, g_order);
    if(g_is_order_publiced == FALSE){
      if(g_order==ORDER_BACK_TO_CHARGE){
        printf("%d\n", g_order);
        play_wav((char*)concat(PACKAGE_PATH, "audios/back_to_charge.wav"));        
      }
      if(g_order == ORDER_FACE_DETECTION){
        printf("%d\n", g_order);
        play_wav((char*)concat(PACKAGE_PATH, "audios/operating_face_rec.wav"));
      }
      g_is_order_publiced = TRUE;
		}
    
  }
exit:
  MSPLogout();
  printf("请按任意键退出...\n");
  getchar();

  return 0;
}




