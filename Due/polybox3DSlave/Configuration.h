#ifndef CONFIGURATION_H
#define CONFIGURATION_H



/***********************************************************************
 *
 * Configuration && Define
 *
 * ********************************************************************/

#define bool boolean

/** Delay timer for execution Loop**/
#define DELAY_EXEC_LOOP         1// 10 //µs, used at the end of each Loop(). Base Timer
#define DELAY_START_UP          1000    //ms  time to wait at the end of setup(), to be sure the main board is ready !
#define DELAY_OFF               1000*30 // = 30sec
#define DELAY_INIT              100
/** delay for slow pin(default). Check if values changed and push then
 * to the updateQueue **/
#define DELAY_CHECK_PIN_SLOW    200 // *timer ( DELAY_MAIN_LOOP )
/** delay for fast pin. Check if values changed and push then
 * to the updateQueue **/
#define DELAY_CHECK_PIN_FAST    20 // *timer ( DELAY_MAIN_LOOP )
/** psuh delay for analog input (use ADC) **/
#define DELAY_PROCESS_ANALOG    1000 // *timer ( DELAY_MAIN_LOOP )


/** we sum X time each analog value, to smooth result (average result) **/
#define SERIAL_BAUDRATE         115200


#define ANALOG_SUM              5

// INTERVAL / (32Khz/128)  = seconds
#define WATCHDOG_INTERVAL       1024u  // 8sec  (~16 seconds max)

/** Output information and state on Serial (rx/tx). **/
#define OUTPUT_SERIAL 1


#endif //CONFIGURATION_H
