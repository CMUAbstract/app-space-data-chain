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

#define WINDOW_SIZE 16
#define SAMPLE_WINDOW_SIZE 112
#define SAMPLE_SIZE 7

/*Get coordinate coor from the sample samp in a window -- window[SAMPGET(0,TEMP)]*/
#define SAMPGET(samp,coor) (SAMPLE_SIZE*samp + coor)

/*Get coordinate coor from the sample samp in window win -- windows[WINGET(0,1,TEMP)*/
#define WINGET(win,samp,coor) (WINDOW_SIZE*SAMPLE_SIZE*win + SAMPLE_SIZE*samp + coor)

#define TEMP 0
#define GX 1
#define GY 2
#define GZ 3
#define MX 4
#define MY 5
#define MZ 6

struct msg_sample{
    CHAN_FIELD_ARRAY(int, sample, SAMPLE_SIZE);
};
#define FIELD_INIT_msg_sample { \
    FIELD_ARRAY_INITIALIZER \
}

struct msg_sample_avg_in{
  CHAN_FIELD(task_t *, next_task);
  CHAN_FIELD_ARRAY(int, window, SAMPLE_WINDOW_SIZE);
};
#define FIELD_INIT_sample_avg_in { \
    FIELD_INITIALIZER \
    FIELD_ARRAY_INITIALIZER(TEMP_WINDOW_SIZE) \
}

struct msg_sample_avg_out{
  CHAN_FIELD_ARRAY(int, average, SAMPLE_SIZE);
};
#define FIELD_INIT_sample_avg_out { \
    FIELD_ARRAY_INITIALIZER(SAMPLE_SIZE) \
}

struct msg_sample_window{
  CHAN_FIELD_ARRAY(int, window, SAMPLE_WINDOW_SIZE);
};
#define FIELD_INIT_msg_reading { \
    FIELD_ARRAY_INITIALIZER(SAMPLE_WINDOW_SIZE) \
}

struct msg_sample_windows{
    SELF_CHAN_FIELD(int, which_window);
    SELF_CHAN_FIELD_ARRAY(int, windows, SAMPLE_WINDOW_SIZE);
    SELF_CHAN_FIELD_ARRAY(int, win_i, TEMP_WINDOW_SIZE);
};
#define FIELD_INIT_msg_sample_windows { \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_ARRAY_INITIALIZER(WINDOW_BUFFER_SIZE), \
    SELF_FIELD_ARRAY_INITIALIZER(TEMP_WINDOW_SIZE) \
}

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
CHANNEL(task_sample, task_window, msg_sample);
SELF_CHANNEL(task_window, msg_index);

/*Window channels to update_window_start*/
CHANNEL(task_window, task_update_window_start, msg_sample_window);
CHANNEL(task_update_window, task_update_window_start, msg_sample_window);

/*Windows channels to task_update_window*/
SELF_CHANNEL(task_update_window, msg_sample_windows);
CHANNEL(task_init, task_update_window, msg_sample_windows);

/*Average takes a window and produces an average*/
CALL_CHANNEL(ch_average, msg_sample_avg_in);
RET_CHANNEL(ch_average, msg_sample_avg_out);


/*This data structure conveys the averages
  to EDB through the callback interface*/

typedef struct _edb_info_t{
  int averages[NUM_WINDOWS];
} edb_info_t;
__attribute__((section(".nv_vars"))) edb_info_t edb_info;

static void write_app_output(uint8_t *output, unsigned *len)
{
    unsigned output_len = NUM_WINDOWS * sizeof(int); // actual app output len
    if( output_len > *len ){ return; }
    memcpy(output, &(edb_info.averages), output_len);
    *len = output_len;
}


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

    WISP_init();

#ifdef CONFIG_EDB
    debug_setup();
#endif

    P1DIR |= BIT0;
   

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

  int ind = TEMP; 
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));

  ind = GX;
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));
  
  ind = GY;
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));
  
  ind = GZ;
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));
  
  ind = MX;
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));
  
  ind = MY;
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));
  
  ind = MZ;
  CHAN_OUT1(int, sample[ind], temp, CH(task_sample, task_window));
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

  int i    = *CHAN_IN1(int, i, SELF_IN_CH(task_window));

  int ind  = 0;
 
  ind = TEMP;
  int temp = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], temp, CH(task_window, task_update_window_start));

  ind = GX;
  int gx = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], gx, CH(task_window, task_update_window_start));

  ind = GY;
  int gy = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], gy, CH(task_window, task_update_window_start));

  ind = GZ;
  int gz = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], gz, CH(task_window, task_update_window_start));

  ind = MX ;
  int mx = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], mx, CH(task_window, task_update_window_start));

  ind = MY ;
  int my = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], my, CH(task_window, task_update_window_start));

  ind = MZ ;
  int mz = *CHAN_IN1(int, sample[ind], CH(task_sample, task_window));
  CHAN_OUT1(int, window[SAMPGET(i,ind)], mz, CH(task_window, task_update_window_start));

  
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

    unsigned i;
    int sum[SAMPLE_SIZE];
    int avg[SAMPLE_SIZE];
    for( i = 0; i < SAMPLE_SIZE; i++ ){
      sum[i] = 0;
      avg[i] = 0;
    }

    int val;
    unsigned j;
    for(i = 0; i < WINDOW_SIZE; i++){
      for(j = 0; j < SAMPLE_SIZE; j++){
        val = *CHAN_IN1(int, window[SAMPGET(i,j)], CALL_CH(ch_average));
        sum[j] += val;
      }
    }

    for(i = 0; i < SAMPLE_SIZE; i++){
      avg[i] = sum[i] >> TEMP_WINDOW_DIV_SHIFT; 
      CHAN_OUT1(int, average[i], avg[i], RET_CH(ch_average));
    }

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
 
  unsigned samp; 
  unsigned coor;
  for(samp = 0; samp < WINDOW_SIZE; samp++){
    for(coor = 0; coor < SAMPLE_SIZE; coor++){

      int temp = *CHAN_IN2(int, window[SAMPGET(samp,coor)], CH(task_window, task_update_window_start),
                                                            CH(task_update_window, task_update_window_start));

      CHAN_OUT1(int, window[SAMPGET(samp,coor)], temp, CALL_CH(ch_average));

    }
    
  }

  TRANSITION_TO(task_average);

}


void task_update_window(){

  /*Get the average and window ID from the averaging call*/
  int avg[SAMPLE_SIZE];
  avg[TEMP] = *CHAN_IN1(int, average[TEMP], RET_CH(ch_average));
  avg[GX] = *CHAN_IN1(int, average[GX], RET_CH(ch_average));
  avg[GY] = *CHAN_IN1(int, average[GY], RET_CH(ch_average));
  avg[GZ] = *CHAN_IN1(int, average[GZ], RET_CH(ch_average));
  avg[MX] = *CHAN_IN1(int, average[MX], RET_CH(ch_average));
  avg[MY] = *CHAN_IN1(int, average[MY], RET_CH(ch_average));
  avg[MZ] = *CHAN_IN1(int, average[MZ], RET_CH(ch_average));
 

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_window),
                                                  SELF_IN_CH(task_update_window));
 
  /*Update the continuously updated average buffer with this average*/ 
  //edb_info.averages[which_window] = avg;

  /*Get the index for this window that we need to update*/
  int win_i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_window), 
                                                  SELF_IN_CH(task_update_window));

  /*Use window ID and win index to self-chan the average, saving it*/
  //WINGET(which_window,win_i,TEMP)
  int ind = which_window * TEMP_WINDOW_SIZE + win_i;
  CHAN_OUT1(int, windows[ind], avg, SELF_OUT_CH(task_update_window));

  /*Send self the next win_i for this window*/
  int next_wini = (win_i + 1) % TEMP_WINDOW_SIZE;
  CHAN_OUT1(int, win_i[which_window], next_wini, SELF_OUT_CH(task_update_window));

  /*Determine the next window to average*/
  int next_window = (which_window + 1) % NUM_WINDOWS;
  CHAN_OUT1(int, which_window, next_window, SELF_OUT_CH(task_update_window));


  /*BRANDON:TODO: rewrite tehse lines to use the expanded data format*/
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
