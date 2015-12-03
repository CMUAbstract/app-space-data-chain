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

struct msg_temp {
    CHAN_FIELD(int, temp);
};

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_report)

CHANNEL(task_sample, task_report, msg_temp);

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

    PRINTF("init: initializing temperature sensor\r\n");
    init_temp_sensor();

    blink0(4,10000000); 

}

void task_init()
{
  
    blink0(1,1000000);
    PRINTF("looping\r\n");
   
    TRANSITION_TO(task_sample);

}

void task_sample(){
  signed short temp = read_temperature_sensor();
  CHAN_OUT1(int, temp, temp, CH(task_sample, task_report));
  TRANSITION_TO(task_report);
}

void task_report(){

  int temp = *CHAN_IN1(int, temp, CH(task_sample, task_report));

  PRINTF("%i/10 deg C\r\n",temp);
  blink0(3,1000000);
  TRANSITION_TO(task_sample);

}

INIT_FUNC(initializeHardware)
ENTRY_TASK(task_init)
