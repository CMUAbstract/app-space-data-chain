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

#define WINDOW_SIZE 16 /*number of samples in a window*/
#define SAMPLE_SIZE 8  /*number of ints in a sample*/
#define SAMPLE_WINDOW_SIZE 128 /*number of ints in a window of samples*/
#define WIN_WINDOW_SIZE 512 /*number of ints in a window of 4 windows of 16 samples of 8 ints each*/

/*Get coordinate coor from the sample samp in a window -- window[SAMPGET(0,TEMP)]*/
#define SAMPGET(samp,coor) (SAMPLE_SIZE*samp + coor)

/*Get coordinate coor from the sample samp in window win -- windows[WINGET(0,1,TEMP)*/
#define WINGET(win,samp,coor) (SAMPLE_WINDOW_SIZE*win + SAMPLE_SIZE*samp + coor)

#define TEMP 0
#define GX 1
#define GY 2
#define GZ 3
#define MX 4
#define MY 5
#define MZ 6
#define ZERO 7

struct msg_sample{
    CHAN_FIELD_ARRAY(int, sample, SAMPLE_SIZE);
};
#define FIELD_INIT_msg_sample { \
    FIELD_ARRAY_INITIALIZER(SAMPLE_SIZE) \
}

struct msg_sample_avg_in{
  CHAN_FIELD(task_t *, next_task);
  CHAN_FIELD_ARRAY(int, window, SAMPLE_WINDOW_SIZE);
};
#define FIELD_INIT_sample_avg_in { \
    FIELD_INITIALIZER \
    FIELD_ARRAY_INITIALIZER(SAMPLE_WINDOW_SIZE) \
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
    CHAN_FIELD(int, which_window);
    CHAN_FIELD_ARRAY(int, win_i, WINDOW_SIZE);
    CHAN_FIELD_ARRAY(int, windows, WIN_WINDOW_SIZE);
};
#define FIELD_INIT_msg_sample_windows { \
    FIELD_INITIALIZER, \
    FIELD_ARRAY_INITIALIZER(WINDOW_SIZE), \
    FIELD_ARRAY_INITIALIZER(WIN_WINDOW_SIZE) \
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
TASK(4, task_update_window_start)
TASK(5, task_update_window)
TASK(6, task_update_proxy)
TASK(7, task_average)

/*Channels to window*/
CHANNEL(task_sample, task_window, msg_sample);

CHANNEL(task_init, task_window, msg_index);
SELF_CHANNEL(task_window, msg_index);

/*Window channels to update_window_start*/
CHANNEL(task_init, task_update_window_start, msg_sample_window);
CHANNEL(task_window, task_update_window_start, msg_sample_window);
CHANNEL(task_update_window, task_update_window_start, msg_sample_window);
CHANNEL(task_update_window_start, task_update_window, msg_sample_avg_out);

/*Windows channels to task_update_window*/
CHANNEL(task_update_proxy,task_update_window, msg_sample_windows);
CHANNEL(task_update_window,task_update_proxy, msg_sample_windows);
CHANNEL(task_init, task_update_window, msg_sample_windows);
CHANNEL(task_init, task_update_proxy, msg_sample_windows);

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


static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

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
 
    
 
    unsigned i;
    int zero = 0;
    for( i = 0; i < NUM_WINDOWS; i++ ){
      /*Zero every window's win_i*/
      CHAN_OUT1(int, win_i[i], zero, CH(task_init, task_update_window));
      CHAN_OUT1(int, win_i[i], zero, CH(task_init, task_update_proxy));
    }
    for( i = 0; i < WIN_WINDOW_SIZE; i++ ){
      int t = 42;
      int t2 = 99;
      CHAN_OUT1(int, windows[i], t, CH(task_init, task_update_window));
      CHAN_OUT1(int, windows[i], t2, CH(task_init, task_update_proxy));
    }
    

    CHAN_OUT1(int, which_window, zero, CH(task_init, task_update_window));
    CHAN_OUT1(int, i, zero, CH(task_init, task_window));
   
    PRINTF("heading to sample\r\n");
    TRANSITION_TO(task_sample);

}

void read_gyro(signed short *x,
               signed short *y,
               signed short *z){
  *x = 10; 
  *y = 20; 
  *z = 30; 
}

void read_mag(signed short *x,
              signed short *y,
              signed short *z){
  *x = 40; 
  *y = 50; 
  *z = 60; 
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
  
  

  signed short val = read_temperature_sensor();

  int ind = TEMP; 
  CHAN_OUT1(int, sample[ind], val, CH(task_sample, task_window));

  signed short gx;
  signed short gy;
  signed short gz;
  read_gyro(&gx,&gy,&gz);

  CHAN_OUT1(int, sample[GX], gx, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[GY], gy, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[GZ], gz, CH(task_sample, task_window));


  signed short mx;
  signed short my;
  signed short mz;
  read_mag(&mx,&my,&mz);
  
  CHAN_OUT1(int, sample[MX], mx, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[MY], my, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[MZ], mz, CH(task_sample, task_window));
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

  

  int i = *CHAN_IN2(int, i, SELF_IN_CH(task_window),
                            CH(task_init, task_window));

  int temp = *CHAN_IN1(int, sample[TEMP], CH(task_sample, task_window));

  int gx = *CHAN_IN1(int, sample[GX], CH(task_sample, task_window));

  int gy = *CHAN_IN1(int, sample[GY], CH(task_sample, task_window));

  int gz = *CHAN_IN1(int, sample[GZ], CH(task_sample, task_window));

  int mx = *CHAN_IN1(int, sample[MX], CH(task_sample, task_window));

  int my = *CHAN_IN1(int, sample[MY], CH(task_sample, task_window));

  int mz = *CHAN_IN1(int, sample[MZ], CH(task_sample, task_window));
  
  
  CHAN_OUT1(int, window[SAMPGET(i,TEMP)], temp, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,GX)], gx, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,GY)], gy, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,GZ)], gz, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,MX)], mx, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,MY)], my, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,MZ)], mz, CH(task_window, task_update_window_start));

  
  int next_i = (i + 1) % TEMP_WINDOW_SIZE;
  CHAN_OUT1(int, i, next_i, SELF_OUT_CH(task_window));


  /*Every TEMP_WINDOW_SIZE samples, compute a new average*/
  if( next_i == 0 ){
    delay(100000);    
    TRANSITION_TO(task_update_window_start);
  }else{
    TRANSITION_TO(task_sample);
  }

}


void task_average(){
    
    

    PRINTF("Entered avg\r\n");
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
      avg[i] = (sum[i]) >> TEMP_WINDOW_DIV_SHIFT; 
      CHAN_OUT1(int, average[i], avg[i], RET_CH(ch_average));
    }

    transition_to(next_task);

}

void printsamp(int t, int gx, int gy, int gz, int mx, int my, int mz){
  PRINTF("{T:%i,G:{%i,%i,%i},M:{%i,%i,%i}}",t,gx,gy,gz,mx,my,mz);
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
  //task_t *next_task = TASK_REF(task_update_window);
  //CHAN_OUT1(task_t *, next_task, next_task, CALL_CH(ch_average));

  unsigned i; 
  unsigned samp; 
  unsigned sum[SAMPLE_SIZE];
  unsigned avg[SAMPLE_SIZE];
  for( i = 0; i < SAMPLE_SIZE; i++ ){
    sum[i] = 0;
  }
  for(samp = 0; samp < WINDOW_SIZE; samp++){


      int val = *CHAN_IN3(int, window[SAMPGET(samp,TEMP)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[TEMP] += val;
      val = *CHAN_IN3(int, window[SAMPGET(samp,GX)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[GX] += val;
      val = *CHAN_IN3(int, window[SAMPGET(samp,GY)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[GY] += val;
      val = *CHAN_IN3(int, window[SAMPGET(samp,GZ)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[GZ] += val;
      val = *CHAN_IN3(int, window[SAMPGET(samp,MX)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[MX] += val;
      val = *CHAN_IN3(int, window[SAMPGET(samp,MY)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[MY] += val;
      val = *CHAN_IN3(int, window[SAMPGET(samp,MZ)], CH(task_window, task_update_window_start),
                                                           CH(task_update_window, task_update_window_start),
                                                           CH(task_init, task_update_window_start));
      sum[MZ] += val;
      
  }
  for( i = 0; i < SAMPLE_SIZE; i++ ){
    avg[i] = sum[i] >> TEMP_WINDOW_DIV_SHIFT;
    CHAN_OUT1(int, average[i], avg[i], CH(task_update_window_start,task_update_window));
  }
 

  TRANSITION_TO(task_update_proxy);

}

void task_update_proxy(){

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_proxy),
                                                  CH(task_update_window,task_update_proxy));

  int i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_proxy), 
                                                  CH(task_update_window,task_update_proxy));

  int temp = *CHAN_IN2(int, windows[WINGET(which_window,i,TEMP)], CH(task_init,task_update_proxy),
                                                                  CH(task_update_window,task_update_proxy));

  int gx = *CHAN_IN2(int, windows[WINGET(which_window,i,GX)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int gy = *CHAN_IN2(int, windows[WINGET(which_window,i,GY)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int gz = *CHAN_IN2(int, windows[WINGET(which_window,i,GZ)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));

  int mx = *CHAN_IN2(int, windows[WINGET(which_window,i,MX)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int my = *CHAN_IN2(int, windows[WINGET(which_window,i,MY)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int mz = *CHAN_IN2(int, windows[WINGET(which_window,i,MZ)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));


  CHAN_OUT1(int, which_window, which_window, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, win_i[which_window], i, CH(task_update_proxy,task_update_window));
  
  CHAN_OUT1(int, windows[WINGET(which_window,i,TEMP)], temp, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,GX)], gx, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,GY)], gy, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,GZ)], gz, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,MX)], mx, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,MY)], my, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,MZ)], mz, CH(task_update_proxy,task_update_window));

  TRANSITION_TO(task_update_window);

}

void task_update_window(){

  /*Get the average and window ID from the averaging call*/
  int avg[SAMPLE_SIZE];
  avg[TEMP] = *CHAN_IN1(int, average[TEMP], CH(task_update_window_start, task_update_window));
  avg[GX] = *CHAN_IN1(int, average[GX], CH(task_update_window_start, task_update_window));
  avg[GY] = *CHAN_IN1(int, average[GY], CH(task_update_window_start, task_update_window));
  avg[GZ] = *CHAN_IN1(int, average[GZ], CH(task_update_window_start, task_update_window));
  avg[MX] = *CHAN_IN1(int, average[MX], CH(task_update_window_start, task_update_window));
  avg[MY] = *CHAN_IN1(int, average[MY], CH(task_update_window_start, task_update_window));
  avg[MZ] = *CHAN_IN1(int, average[MZ], CH(task_update_window_start, task_update_window));

  PRINTF("from avg got ");
  printsamp(avg[TEMP],avg[GX],avg[GY],avg[GZ],avg[MX],avg[MY],avg[MZ]);
  PRINTF("\r\n");
 

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_window),
                                                  CH(task_update_proxy,task_update_window));

  PRINTF("Working with window %i\r\n",which_window); 
  /*Update the continuously updated average buffer with this average*/ 
  //edb_info.averages[which_window] = avg;

  /*Get the index for this window that we need to update*/
  int win_i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_window), 
                                                  CH(task_update_proxy,task_update_window));
  PRINTF("Working with window bin %i\r\n",win_i); 

  /*Use window ID and win index to self-chan the average, saving it*/
  //WINGET(which_window,win_i,TEMP)
  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + TEMP], avg[TEMP], CH(task_update_window,task_update_proxy));

  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + GX], avg[GX], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + GY], avg[GY], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + GZ], avg[GZ], CH(task_update_window,task_update_proxy));
  
  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + MX], avg[MX], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + MY], avg[MY], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[which_window*SAMPLE_WINDOW_SIZE + win_i*SAMPLE_SIZE + MZ], avg[MZ], CH(task_update_window,task_update_proxy));
  /*Send self the next win_i for this window*/
  int next_wini = (win_i + 1) % WINDOW_SIZE;
  //CHAN_OUT1(int, win_i[which_window], next_wini, SELF_OUT_CH(task_update_window));
  CHAN_OUT1(int, win_i[which_window], next_wini, CH(task_update_window,task_update_proxy));

  /*Determine the next window to average*/
  int next_window = (which_window + 1) % NUM_WINDOWS;
  //CHAN_OUT1(int, which_window, next_window, SELF_OUT_CH(task_update_window));
  CHAN_OUT1(int, which_window, next_window, CH(task_update_window,task_update_proxy));

  if( next_window != 0 ){
  /*Not the last window*/
 
    /*For pretty printing*/ 
    unsigned s; 
    for( s = 0; s < which_window + 1; s++ ){
      PRINTF(" ");
    }
    PRINTF("[");
    
    /*Put this average in the next window, 
      then re-average that window*/
    unsigned i;
    for(i = 0; i < WINDOW_SIZE; i++){

      if( i == win_i ){
        /*window[win_i] that goes back to task_update_window_start gets the avg*/
 
        PRINTF("W");
        printsamp(avg[TEMP],avg[GX],avg[GY],avg[GZ],avg[MX],avg[MY],avg[MZ]);

        CHAN_OUT1(int, window[SAMPGET(i,TEMP)], avg[TEMP], CH(task_update_window, task_update_window_start));
        
        CHAN_OUT1(int, window[SAMPGET(i,GX)], avg[GX], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GY)], avg[GY], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GZ)], avg[GZ], CH(task_update_window, task_update_window_start));

        CHAN_OUT1(int, window[SAMPGET(i,MX)], avg[MX], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MY)], avg[MY], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MZ)], avg[MZ], CH(task_update_window, task_update_window_start));


      }else{
        /*window[i != win_i] that goes back to task_update_window_start gets self's window[i]*/

        PRINTF("O");
        /*In the value from the self channel*/
        int temp = *CHAN_IN2(int, windows[WINGET(which_window,i,TEMP)], CH(task_init,task_update_window),
                                                                        CH(task_update_proxy,task_update_window));

        int gx = *CHAN_IN2(int, windows[WINGET(which_window,i,GX)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int gy = *CHAN_IN2(int, windows[WINGET(which_window,i,GY)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int gz = *CHAN_IN2(int, windows[WINGET(which_window,i,GZ)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));

        int mx = *CHAN_IN2(int, windows[WINGET(which_window,i,MX)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int my = *CHAN_IN2(int, windows[WINGET(which_window,i,MY)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int mz = *CHAN_IN2(int, windows[WINGET(which_window,i,MZ)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));

        printsamp(temp,gx,gy,gz,mx,my,mz);

        /*Out the value back to task_update_window_start for averaging*/
        CHAN_OUT1(int, window[SAMPGET(i,TEMP)], temp, CH(task_update_window, task_update_window_start));
        
        CHAN_OUT1(int, window[SAMPGET(i,GX)], gx, CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GY)], gy, CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GZ)], gz, CH(task_update_window, task_update_window_start));

        CHAN_OUT1(int, window[SAMPGET(i,MX)], mx, CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MY)], my, CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MZ)], mz, CH(task_update_window, task_update_window_start));

      }

    }
    PRINTF("]\r\n");

    TRANSITION_TO(task_update_window_start);

  }else{
  /*The last window: go back to sampling*/

    /*Done with all windows!*/
    TRANSITION_TO(task_sample);

  }

}

INIT_FUNC(initializeHardware)
ENTRY_TASK(task_init)
