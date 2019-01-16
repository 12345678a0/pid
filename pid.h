#ifndef		__PID_H__
#define		__PID_H__

#include "configPid.h"


void pid_init();
void pid_handle(); 
void pid_run_tick(uint8_t base);


#endif /*__PID_CONTROL_H__*/

