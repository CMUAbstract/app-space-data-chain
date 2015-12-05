#include <msp430.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>

#include <msp-builtins.h>
#include <msp-math.h>

#include <libmsp/mem.h>
#include <libio/log.h>
#include <libchain/chain.h>

#ifdef CONFIG_LIBEDB_PRINTF
#include <libedb/edb.h>
#endif

#include "pins.h"
#include "temp_sensor.h"

// #define SHOW_RESULT_ON_LEDS
// #define SHOW_PROGRESS_ON_LEDS
// #define SHOW_BOOT_ON_LEDS

// Number of samples to discard before recording training set
#define NUM_WARMUP_SAMPLES 3

#define ACCEL_WINDOW_SIZE 3
#define MODEL_SIZE 16
#define SAMPLE_NOISE_FLOOR 10 // TODO: made up value

// Number of classifications to complete in one experiment
#define SAMPLES_TO_COLLECT 128

#define SEC_TO_CYCLES 4000000 /* 4 MHz */

#define IDLE_WAIT SEC_TO_CYCLES

#define IDLE_BLINKS 1
#define IDLE_BLINK_DURATION SEC_TO_CYCLES
#define SELECT_MODE_BLINKS  4
#define SELECT_MODE_BLINK_DURATION  (SEC_TO_CYCLES / 5)
#define SAMPLE_BLINKS  1
#define SAMPLE_BLINK_DURATION  (SEC_TO_CYCLES * 2)
#define FEATURIZE_BLINKS  2
#define FEATURIZE_BLINK_DURATION  (SEC_TO_CYCLES * 2)
#define CLASSIFY_BLINKS 1
#define CLASSIFY_BLINK_DURATION (SEC_TO_CYCLES * 4)
#define WARMUP_BLINKS 2
#define WARMUP_BLINK_DURATION (SEC_TO_CYCLES / 2)
#define TRAIN_BLINKS 1
#define TRAIN_BLINK_DURATION (SEC_TO_CYCLES * 4)

#define LED1 (1 << 0)
#define LED2 (1 << 1)

#define TEMP_WINDOW_SIZE 16
#define TEMP_WINDOW_DIV_SHIFT 4 
#define NUM_WINDOWS 4

struct msg_temp {
    CHAN_FIELD(int, temp);
};
#define FIELD_INIT_msg_temp { \
    FIELD_INITIALIZER \
}

struct msg_temp_window{
    CHAN_FIELD(int, avg_now);
    CHAN_FIELD_ARRAY(int, window, TEMP_WINDOW_SIZE);
};
#define FIELD_INIT_msg_temp_window { \
    FIELD_ARRAY_INITIALIZER(TEMP_WINDOW_SIZE) \
}

struct msg_index{
    SELF_CHAN_FIELD(int, i);
};
#define FIELD_INIT_msg_index { \
    SELF_FIELD_INITIALIZER \
}

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_window)
TASK(4, task_report)

/*Channels to window*/
CHANNEL(task_sample, task_window, msg_temp);
SELF_CHANNEL(task_window, msg_index);

/*Channels to report*/
CHANNEL(task_window, task_report, msg_temp_window);
CHANNEL(task_init, task_report, msg_temp_window);

#if defined(SHOW_RESULT_ON_LEDS) || defined(SHOW_PROGRESS_ON_LEDS)
static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

static void blink(unsigned count, uint32_t duration, unsigned leds)
{
    unsigned i;
    for (i = 0; i < count; ++i) {
        GPIO(PORT_LED_1, OUT) |= (leds & LED1) ? BIT(PIN_LED_1) : 0x0;
        GPIO(PORT_LED_2, OUT) |= (leds & LED2) ? BIT(PIN_LED_2) : 0x0;
        delay(duration / 2);
        GPIO(PORT_LED_1, OUT) &= (leds & LED1) ? ~BIT(PIN_LED_1) : ~0x0;
        GPIO(PORT_LED_2, OUT) &= (leds & LED2) ? ~BIT(PIN_LED_2) : ~0x0;
        delay(duration / 2);
    }
}
#endif
static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

static void blink0(unsigned count, uint32_t duration){
    unsigned i;
    for (i = 0; i < count; ++i) {
        P1OUT ^= BIT0;
        delay(duration);
    }

}


void initializeHardware()
{

    WISP_init();

#ifdef CONFIG_EDB
    debug_setup();
#endif

    P1DIR |= BIT0;
   
    blink0(1,10000000); 

    INIT_CONSOLE();
    
    __enable_interrupt();

    blink0(1,10000000); 

}

/*Initialize the sample window
  Input channels: 
      none
  Output channels: 
    { int window[TEMP_WINDOW_SIZE}
      initialize the window 
  Successors:
      task_sample
*/
void task_init()
{
  
    blink0(1,1000000);
    PRINTF("initializing the window\r\n");
   
    unsigned i;
    for( i = 0; i < TEMP_WINDOW_SIZE; i++ ){
      int t = -99;
      CHAN_OUT1(int, window[i], t, CH(task_init,task_report));
    }
   
    TRANSITION_TO(task_sample);

}

/*Collect the next temperature sample
  Input channels: 
      none
  Output channels: 
    { int temp }
      send the next temperature reading to put it in the window
  Successors:
      task_window
*/
void task_sample(){

  signed short temp = read_temperature_sensor();
  CHAN_OUT1(int, temp, temp, CH(task_sample, task_window));
  TRANSITION_TO(task_window);

}


/*Report the samples in the window
  Input channels: 
    { int i; }
      self channel sends window index 
    { int temp; }
      receive a temperature reading from task_sample 
  Output channels: 
    { int window[TEMP_WINDOW_SIZE] }
      send the next temperature reading to task_report in the window
  Successors:
      task_report
*/
void task_window(){

  int temp = *CHAN_IN1(int, temp, CH(task_sample, task_window));
  int i    = *CHAN_IN1(int, i, SELF_IN_CH(task_window));

  PRINTF("Putting %i in the window at position %i\r\n",temp,i); 
  CHAN_OUT1(int, window[i], temp, CH(task_window, task_report));

  /*This sample fit in the window*/
  i = (i + 1) % TEMP_WINDOW_SIZE;
  CHAN_OUT1(int, i, i, SELF_OUT_CH(task_window));

  if( i < TEMP_WINDOW_SIZE ){
    int a = 0;
    CHAN_OUT1(int, avg_now, a, CH(task_window, task_report));
  }else{
    int a = 1;
    CHAN_OUT1(int, avg_now, a, CH(task_window, task_report));
  }
  TRANSITION_TO(task_report);

}

/*Report the samples in the window
  Input channels: 
    { int window[TEMP_WINDOW_SIZE]; }
      task_init sends initial values of window
      task_window sends input values that it puts in the window
  Output channels: none
  Successors:
      task_sample
*/
void task_report(){

  int avg_now = *CHAN_IN1(int, avg_now, CH(task_window, task_report));
  if( avg_now ){

    int sum = 0;
    PRINTF("[");
    unsigned i;
    int temp;
    for(i = 0; i < TEMP_WINDOW_SIZE-1; i++){
      temp = *CHAN_IN2(int, window[i], CH(task_window, task_report),
                                       CH(task_init, task_report));
      sum += temp;
      PRINTF("%i, ",temp);
    }
    temp = *CHAN_IN2(int, window[TEMP_WINDOW_SIZE-1], CH(task_window, task_report),
                                                      CH(task_init, task_report));
    sum += temp;
    PRINTF("%i] ",temp);
 
    int avg = sum >> TEMP_WINDOW_DIV_SHIFT; 
    PRINTF("(avg: %i\r\n",avg);

    blink0(5,1000000);
  
  }
  /*CHOUT is[NUM_WINDOWS]*/
  /*Iterate over them in here, 
    and as they are full, re-average them*/

  TRANSITION_TO(task_sample);

}

INIT_FUNC(initializeHardware)
ENTRY_TASK(task_init)
