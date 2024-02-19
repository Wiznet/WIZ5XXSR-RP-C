#ifndef TIMERHANDLER_H_
#define TIMERHANDLER_H_

#include <sys/time.h>
#include <stdint.h>

// For main routine checker
//#define MAIN_ROUTINE_CHECK_CYCLE_MSEC		100 // msec
//#define DEFINED_COUNT_THRESHOLD_VAL			35

//extern uint8_t flag_check_main_routine;

void Timer_Configuration(void);
void delay_ms(uint32_t ms);

////////////////////////////////////////
time_t getNow(void);
time_t getDevtime(void);
void setNow(time_t timeval_sec);

time_t getDevtime(void);
void setDevtime(time_t timeval_sec);

time_t millis(void);
////////////////////////////////////////

uint32_t getDeviceUptime_day(void);
uint8_t getDeviceUptime_hour(void);
uint8_t  getDeviceUptime_min(void);
uint8_t  getDeviceUptime_sec(void);
uint16_t getDeviceUptime_msec(void);

#endif /* TIMERHANDLER_H_ */
