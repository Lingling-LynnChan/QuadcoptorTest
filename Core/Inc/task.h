#ifndef __GW_TASK_H__
#define __GW_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

void GW_Task_1ms(void);
void GW_Ctrl_Flight(float dt);
void GW_Ctrl_Motor(void);

#ifdef __cplusplus
}
#endif

#endif /*__GW_TASK_H__ */