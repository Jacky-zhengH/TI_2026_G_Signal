#ifndef __APP_PROCESS_H
#define __APP_PROCESS_H

/*----标准库文件----*/
#include "stdint.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stdarg.h"
/*----stm32 f4 hal库----*/
#include "stm32f4xx_hal.h"
//=======================================
// 函数声明
//=======================================
void HMI_Process_Init(void);
void HMI_Send_Cmd(const char *cmd_string);
void Debug_printf(const char *text, ...);
void App_Main_Process_Poll(void);

#endif
