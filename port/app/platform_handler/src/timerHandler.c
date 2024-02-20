
#include "common.h"
#include "WIZ5XXSR-RP_board.h"
#include "timerHandler.h"
#include "seg.h"
#include "segcp.h"
#include "deviceHandler.h"
#include "gpioHandler.h"

#include "dhcp.h"
#include "dns.h"

bool repeating_timer_callback(struct repeating_timer *t) ;

volatile uint32_t delaytime_msec = 0;

static volatile uint16_t msec_cnt = 0;
static volatile uint8_t  sec_cnt = 0;
static volatile uint8_t  min_cnt = 0;
static volatile uint8_t  hour_cnt = 0;
static volatile uint16_t day_cnt = 0;

static volatile time_t currenttime_sec = 0;
static volatile time_t devtime_sec = 0;
static volatile time_t devtime_msec = 0;

extern uint8_t factory_flag;
static uint32_t factory_time;

extern uint8_t ssl_handshake_flag;

extern uint8_t flag_s2e_application_running;
struct repeating_timer timer;

void Timer_Configuration(void)
{
    add_repeating_timer_us(-1000, repeating_timer_callback, NULL, &timer);
}

bool repeating_timer_callback(struct repeating_timer *t) 
{
    delaytime_msec++;
    msec_cnt++; // millisecond counter

    devtime_msec++;

    seg_timer_msec();		        // [msec] time counter for SEG (S2E)
    segcp_timer_msec();		        // [msec] time counter for SEGCP (Config)
    //device_timer_msec();	        // [msec] time counter for DeviceHandler (fw update)
    MilliTimer_Handler();           // [msec] time counter for MQTT client

    gpio_handler_timer_msec();

    /* Second Process */
    if(msec_cnt >= 1000 - 1)
    {
        msec_cnt = 0;
        sec_cnt++;                  // second counter

        seg_timer_sec();            // [sec] time counter for SEG (S2E)

        DHCP_time_handler();	    // [sec] time counter for DHCP timeout
        DNS_time_handler();		    // [sec] time counter for DNS timeout

        devtime_sec++;              // device time counter,
        currenttime_sec++;          // Can be updated this counter value by time protocol like NTP.
        LED_Toggle(LED3);

#ifdef __USE_S2E_OVER_TLS__
    if(ssl_handshake_flag) {
        watchdog_update();
    }
#endif
    }

    /* Minute Process */
    if(sec_cnt >= 60)
    {
        sec_cnt = 0;
        min_cnt++;                  // minute counter
    }

    /* Hour Process */
    if(min_cnt >= 60)
    {
        min_cnt = 0;
        hour_cnt++;                 // hour counter
    }

    /* Day Process */
    if(hour_cnt >= 24)
    {
        hour_cnt = 0;
        day_cnt++;                  // day counter
    }

#ifdef __USE_HW_FACTORY_RESET__
    /* Factory Reset Process */
    if(factory_flag) {
        factory_time++;
        if (get_factory_reset_pin())
        {
            factory_flag = 0;
            factory_time = 0;
        }
        else if (factory_time >= FACTORY_RESET_TIME_MS) 
        {
            /* Factory Reset */
            device_set_factory_default();
            NVIC_SystemReset();
        }
    }
#endif        

    return true;
}

void delay_ms(uint32_t ms)
{
    uint32_t wakeuptime_msec = delaytime_msec + ms;
    while(wakeuptime_msec > delaytime_msec){}
}

time_t getDevtime(void)
{
    return devtime_sec;
}

void setDevtime(time_t timeval_sec)
{
    devtime_sec = timeval_sec;
}

time_t millis(void)
{
    return devtime_msec;
}

time_t getNow(void)
{
    return currenttime_sec;
}

void setNow(time_t timeval_sec)
{
    currenttime_sec = timeval_sec;
}

uint32_t getDeviceUptime_day(void)
{
    return day_cnt;
}

uint8_t getDeviceUptime_hour(void)
{
    return hour_cnt;
}

uint8_t getDeviceUptime_min(void)
{
    return min_cnt;
}

uint8_t getDeviceUptime_sec(void)
{
    return sec_cnt;
}

uint16_t getDeviceUptime_msec(void)
{
    return msec_cnt;
}
