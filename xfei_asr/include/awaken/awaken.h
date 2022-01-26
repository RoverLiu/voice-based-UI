#ifndef _AWAKEN_H_
#define _AWAKEN_H_
// #include "asr_record/asr_record.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum{FALSE=0,TRUE}BOOL;
extern void run_ivw(const char *grammar_list ,  const char* session_begin_params);
extern BOOL g_is_awaken_succeed;

#ifdef __cplusplus
}
#endif



#endif