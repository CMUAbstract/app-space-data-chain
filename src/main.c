#include <msp430.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

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

#define TEMP_WINDOW_SIZE 16
#define TEMP_WINDOW_DIV_SHIFT 4 
#define NUM_WINDOWS 4
#define WINDOW_BUFFER_SIZE 64 /*TEMP_WINDOW_SIZE * NUM_WINDOWS: */
                              /*to help the macro system figure out channeling*/

struct msg_index{
    SELF_CHAN_FIELD(int, i);
};
#define FIELD_INIT_msg_index { \
    SELF_FIELD_INITIALIZER \
}

struct msg_temp {
    CHAN_FIELD(int, temp);
};
#define FIELD_INIT_msg_temp { \
    FIELD_INITIALIZER \
}

struct msg_temp_window{
    CHAN_FIELD_ARRAY(int, window, TEMP_WINDOW_SIZE);
};
#define FIELD_INIT_msg_temp_window { \
    FIELD_ARRAY_INITIALIZER(TEMP_WINDOW_SIZE) \
}

struct msg_avg_in{
    CHAN_FIELD(task_t *, next_task);
    CHAN_FIELD_ARRAY(int, window, TEMP_WINDOW_SIZE);
};
#define FIELD_INIT_msg_avg_in { \
    FIELD_INITIALIZER, \
    FIELD_ARRAY_INITIALIZER(TEMP_WINDOW_SIZE) \
}

struct msg_avg_out{
    CHAN_FIELD(int, average);
};
#define FIELD_INIT_msg_avg_out { \
    FIELD_INITIALIZER \
}

struct msg_temp_windows{
    SELF_CHAN_FIELD(int, which_window);
    SELF_CHAN_FIELD_ARRAY(int, windows, WINDOW_BUFFER_SIZE);
    SELF_CHAN_FIELD_ARRAY(int, win_i, TEMP_WINDOW_SIZE);
};
#define FIELD_INIT_msg_temp_windows { \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_ARRAY_INITIALIZER(WINDOW_BUFFER_SIZE), \
    SELF_FIELD_ARRAY_INITIALIZER(TEMP_WINDOW_SIZE) \
}


TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_window)
TASK(4, task_update_window_start)
TASK(5, task_update_window)
TASK(6, task_average)

/*Channels to window*/
CHANNEL(task_sample, task_window, msg_temp);
SELF_CHANNEL(task_window, msg_index);

/*Window channels to update_window_start*/
CHANNEL(task_window, task_update_window_start, msg_temp_window);
CHANNEL(task_update_window, task_update_window_start, msg_temp_window);

/*Windows channels to task_update_window*/
SELF_CHANNEL(task_update_window, msg_temp_windows);
CHANNEL(task_init, task_update_window, msg_temp_windows);

/*Average takes a window and produces an average*/
CALL_CHANNEL(ch_average, msg_avg_in);
RET_CHANNEL(ch_average, msg_avg_out);


/*This data structure conveys the averages
  to EDB through the callback interface*/

typedef struct _edb_info_t{
  int averages[NUM_WINDOWS];
} edb_info_t;

__nv edb_info_t edb_info;

#ifdef CONFIG_EDB
static void write_app_output(uint8_t *output, unsigned *len)
{
    unsigned output_len = NUM_WINDOWS * sizeof(int); // actual app output len
    if( output_len > *len ){ return; }
    memcpy(output, &(edb_info.averages), output_len);
    *len = output_len;
}
#endif // CONFIG_EDB


#ifdef false
static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}
#endif

void initializeHardware()
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

#if defined(BOARD_EDB) || defined(BOARD_WISP) || defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    PM5CTL0 &= ~LOCKLPM5;	   // Enable GPIO pin settings
#endif

#if defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    CSCTL0_H = 0xA5;
    CSCTL1 = DCOFSEL_6; //8MHz
    CSCTL3 = DIVA_0 + DIVS_0 + DIVM_0;
#endif

#ifdef CONFIG_EDB
    debug_setup();
    edb_set_app_output_cb(write_app_output);
#endif

    INIT_CONSOLE();

    __delay_cycles(100000);
    
    __enable_interrupt();


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
  
    PRINTF("initializing the window\r\n");
    unsigned i;
    unsigned j;
    for( i = 0; i < NUM_WINDOWS; i++ ){
      int win_i = 0;
      CHAN_OUT1(int, win_i[i], win_i, CH(task_init, task_update_window));
      for( j = 0; j < TEMP_WINDOW_SIZE; j++ ){
        int t = -99;
        CHAN_OUT1(int, windows[i*TEMP_WINDOW_SIZE + j], t, CH(task_init, task_update_window));
      }
    }
    int zero = 0;
    CHAN_OUT1(int, which_window, zero, CH(task_init, task_update_window));
   
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
      send the next temperature reading to task_update_window_start in the window
  Successors:
      task_update_window_start
*/
void task_window(){

  int temp = *CHAN_IN1(int, temp, CH(task_sample, task_window));
  int i    = *CHAN_IN1(int, i, SELF_IN_CH(task_window));

  CHAN_OUT1(int, window[i], temp, CH(task_window, task_update_window_start));

  int next_i = (i + 1) % TEMP_WINDOW_SIZE;
  CHAN_OUT1(int, i, next_i, SELF_OUT_CH(task_window));

  /*Every TEMP_WINDOW_SIZE samples, compute a new average*/
  if( next_i == 0 ){
    TRANSITION_TO(task_update_window_start);
  }else{
    TRANSITION_TO(task_sample);
  }

}


void task_average(){

    task_t *next_task = *CHAN_IN1(task_t *, next_task, CALL_CH(ch_average));

    int sum = 0;
    PRINTF("[");
    unsigned i;
    int temp;
    for(i = 0; i < TEMP_WINDOW_SIZE-1; i++){
      temp = *CHAN_IN1(int, window[i], CALL_CH(ch_average));
      sum += temp;
      PRINTF("%i, ",temp);
    }
    temp = *CHAN_IN1(int, window[TEMP_WINDOW_SIZE-1], CALL_CH(ch_average));
    sum += temp;

    PRINTF("%i] ",temp);
 
    int avg = sum >> TEMP_WINDOW_DIV_SHIFT; 
    PRINTF("(avg: %i)\r\n",avg);
    
    CHAN_OUT1(int, average, avg, RET_CH(ch_average));

    transition_to(next_task);

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
void task_update_window_start(){

  /*average needs to know the successor task*/
  task_t *next_task = TASK_REF(task_update_window);
  CHAN_OUT1(task_t *, next_task, next_task, CALL_CH(ch_average));
  
  
  for(int i = 0; i < TEMP_WINDOW_SIZE; i++){ 
    int temp = *CHAN_IN2(int, window[i], CH(task_window, task_update_window_start),
                                         CH(task_update_window, task_update_window_start));

    CHAN_OUT1(int, window[i], temp, CALL_CH(ch_average));
    
  }

  TRANSITION_TO(task_average);

}


void task_update_window(){

  /*Get the average and window ID from the averaging call*/
  int avg = *CHAN_IN1(int, average, RET_CH(ch_average));
 

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_window),
                                                  SELF_IN_CH(task_update_window));
 
  /*Update the continuously updated average buffer with this average*/ 
  edb_info.averages[which_window] = avg;

  /*Get the index for this window that we need to update*/
  int win_i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_window), 
                                                  SELF_IN_CH(task_update_window));

  /*Use window ID and win index to self-chan the average, saving it*/
  int ind = which_window * TEMP_WINDOW_SIZE + win_i;
  CHAN_OUT1(int, windows[ind], avg, SELF_OUT_CH(task_update_window));

  /*Send self the next win_i for this window*/
  int next_wini = (win_i + 1) % TEMP_WINDOW_SIZE;
  CHAN_OUT1(int, win_i[which_window], next_wini, SELF_OUT_CH(task_update_window));

  /*Determine the next window to average*/
  int next_window = (which_window + 1) % NUM_WINDOWS;
  CHAN_OUT1(int, which_window, next_window, SELF_OUT_CH(task_update_window));

  if( next_window != 0 ){
  /*Not the last window*/
 
    /*For pretty printing*/ 
    unsigned s; 
    for( s = 0; s < which_window + 1; s++ ){
      PRINTF(" ");
    }
    
    /*Put this average in the next window, 
      then re-average that window*/
    unsigned i;
    for(i = 0; i < TEMP_WINDOW_SIZE; i++){

      if( i == win_i ){
        /*window[win_i] that goes back to task_update_window_start gets the avg*/

        CHAN_OUT1(int, window[i], avg, CH(task_update_window, task_update_window_start));

      }else{
        /*window[i != win_i] that goes back to task_update_window_start gets self's window[i]*/

        /*Compute the index, reusing ind*/ 
        ind = which_window * TEMP_WINDOW_SIZE + i;
  
        /*In the value from the self channel*/
        int val = *CHAN_IN1(int, windows[ind], SELF_IN_CH(task_update_window));
        
        /*Out the value back to task_update_window_start for averaging*/
        CHAN_OUT1(int, window[i], val, CH(task_update_window, task_update_window_start));

      }

    }

    TRANSITION_TO(task_update_window_start);

  }else{
  /*The last window: go back to sampling*/

    /*Done with all windows!*/
    TRANSITION_TO(task_sample);

  }

}

INIT_FUNC(initializeHardware)
ENTRY_TASK(task_init)
