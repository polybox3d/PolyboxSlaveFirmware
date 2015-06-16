#ifndef __HAL_H__
#define __HAL_H__

void setup_analog_timer()
{/*
    TCCR0A = 0;  // Setup analog interrupt
    OCR0B = 64;
    TIMSK0 |= (1<<OCIE0B);*/
}

inline static void startWatchdog()
{
    WDT->WDT_MR = WDT_MR_WDRSTEN | WATCHDOG_INTERVAL | (WATCHDOG_INTERVAL << 16);
    WDT->WDT_CR = 0xA5000001;
};
inline static void stopWatchdog() {}
inline static void pingWatchdog()
{
    WDT->WDT_CR = 0xA5000001;
};


#endif //__HAL_H__
