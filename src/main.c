#include <msp430.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <libio/log.h>
#include <libchain/chain.h>
#include <libwispbase/wisp-base.h>

#include "pins.h"
#include "temp_sensor.h"
#include "mspware/driverlib.h"
#include "magnetometer.h"
#include "gyro.h"

// Must be after any header that includes mps430.h due to
// the workround of undef'ing 'OUT' (see pin_assign.h)
#ifdef CONFIG_EDB
#include <libedb/edb.h>
#else
#define WATCHPOINT(...)
#endif

// #define SHOW_RESULT_ON_LEDS
// #define SHOW_PROGRESS_ON_LEDS
// #define SHOW_BOOT_ON_LEDS
#define BOARD_SPRITE_APP

#define WINDOW_DIV_SHIFT 2
#define NUM_WINDOWS 4

#define WINDOW_SIZE 4 /*number of samples in a window*/
#define SAMPLE_SIZE 8  /*number of ints in a sample*/
#define SAMPLE_WINDOW_SIZE 32 /*number of ints in a window of samples*/
#define WIN_WINDOW_SIZE 128 /*number of ints in a window of 4 windows of 16 samples of 8 ints each*/

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

struct msg_sample_avg_in{
  CHAN_FIELD(task_t *, next_task);
  CHAN_FIELD_ARRAY(int, window, SAMPLE_WINDOW_SIZE);
};

struct msg_sample_avg_out{
  CHAN_FIELD_ARRAY(int, average, SAMPLE_SIZE);
};

struct msg_sample_window{
  CHAN_FIELD_ARRAY(int, window, SAMPLE_WINDOW_SIZE);
};


struct msg_sample_windows{
    CHAN_FIELD(int, which_window);
    CHAN_FIELD_ARRAY(int, win_i, WINDOW_SIZE);
    CHAN_FIELD_ARRAY(int, windows, WIN_WINDOW_SIZE);
};

struct msg_index{
    CHAN_FIELD(int, i);
};

struct msg_self_index{
    SELF_CHAN_FIELD(int, i);
};
#define FIELD_INIT_msg_self_index { \
    SELF_FIELD_INITIALIZER \
}


TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_window)
TASK(4, task_update_window_start)
TASK(5, task_update_window)
TASK(6, task_update_proxy)

/*Channels to window*/
CHANNEL(task_sample, task_window, msg_sample);

CHANNEL(task_init, task_window, msg_index);
SELF_CHANNEL(task_window, msg_self_index);

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

#define WATCHPOINT_BOOT                 0
#define WATCHPOINT_SAMPLE               1
#define WATCHPOINT_WINDOW               2
#define WATCHPOINT_UPDATE_WINDOW_START  3
//#define WATCHPOINT_UPDATE_WINDOW       4

/*This data structure conveys the averages
  to EDB through the callback interface*/

typedef struct _samp_t{
  int8_t temp;
  int8_t gx;
  int8_t gy;
  int8_t gz;
  int8_t mx;
  int8_t my;
  int8_t mz;
} samp_t;
  
typedef struct _edb_info_t{
  samp_t averages[NUM_WINDOWS];
} edb_info_t;

__nv edb_info_t edb_info;

void i2c_setup(void) {
  /*
  * Select Port 1
  * Set Pin 6, 7 to input Secondary Module Function:
  *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL)
  */


  GPIO_setAsPeripheralModuleFunctionInputPin(
    GPIO_PORT_P1,
    GPIO_PIN6 + GPIO_PIN7,
    GPIO_SECONDARY_MODULE_FUNCTION
  );



  EUSCI_B_I2C_initMasterParam param = {0};
  param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
  param.i2cClk = CS_getSMCLK();
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_400KBPS;
  param.byteCounterThreshold = 0;
  param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;

  EUSCI_B_I2C_initMaster(EUSCI_B0_BASE, &param);
  

}

#ifdef CONFIG_EDB
static void write_app_output(uint8_t *output, unsigned *len)
{
    unsigned output_len = sizeof(edb_info.averages); // actual app output len
    if( output_len > *len ) {
        PRINTF("output app data buf too small: need %u have %u\r\n",
                output_len, *len);
        return;
    }

#if VERBOSE > 0
    int i;
    BLOCK_LOG_BEGIN();
    BLOCK_LOG("handling EDB request for app output:\r\n");
    for (i = 0; i < output_len; ++i) {
        BLOCK_LOG("0x%02x ", *(((uint8_t *)&edb_info.averages) + i));
        if (((i + 1) & (8 - 1)) == 0)
            BLOCK_LOG("\r\n");
    }
    BLOCK_LOG("\r\n");
    BLOCK_LOG_END();
#endif

    memcpy(output, &(edb_info.averages), output_len);
    *len = output_len;
}
#endif // CONFIG_EDB

static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

void initializeHardware()
{
    WDTCTL = WDTPW | WDTHOLD;  // Stop watchdog timer

#if defined(BOARD_EDB) || defined(BOARD_WISP) || defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    PM5CTL0 &= ~LOCKLPM5;	   // Enable GPIO pin settings
#endif

#if defined(BOARD_SPRITE_APP_SOCKET_RHA) || defined(BOARD_SPRITE_APP)
    P1DIR |= BIT0 | BIT1 | BIT2;
    P1OUT &= ~(BIT0 | BIT1 | BIT2);
    P2DIR |= BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7;
    P2OUT &= ~(BIT2 | BIT3 | BIT4 | BIT5 | BIT6 | BIT7);
    P3DIR |= BIT6 | BIT7;
    P3OUT &= ~(BIT6 | BIT7);
    P4DIR |= BIT0 | BIT1 | BIT4;
    P4OUT &= ~(BIT0 | BIT1 | BIT4);
    PJDIR |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
    PJOUT |= BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5;
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

    __enable_interrupt();

    WATCHPOINT(WATCHPOINT_BOOT);

    i2c_setup();

    magnetometer_init();

    gyro_init();

    LOG("space app: curtsk %u\r\n", curctx->task->idx);
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
    LOG("Space Data App Initializing\r\n");

    /* Init data buffer that will contain averages to be fetched by EDB */
    memset(&edb_info, 0, sizeof(edb_info));
  
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

    TRANSITION_TO(task_sample);
}

void read_gyro(int *x,
               int *y,
               int *z){
  gyro_t co;
  gyro_read(&co);
  *x = co.x; 
  *y = co.y; 
  *z = co.z; 
}


void read_mag(int *x,
              int *y,
              int *z){
  magnet_t co;
  magnetometer_read(&co);
  *x = co.x; 
  *y = co.y; 
  *z = co.z; 
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
  
  
  WATCHPOINT(WATCHPOINT_SAMPLE);


  signed short val = read_temperature_sensor();

  int ind = TEMP; 
  CHAN_OUT1(int, sample[ind], val, CH(task_sample, task_window));

  
  gyro_t gd;
  read_gyro(&(gd.x),&(gd.y),&(gd.z));

  CHAN_OUT1(int, sample[GX], gd.x, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[GY], gd.y, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[GZ], gd.z, CH(task_sample, task_window));



  magnet_t md;
  read_mag(&(md.x),&(md.y),&(md.z));
  
  CHAN_OUT1(int, sample[MX], md.x, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[MY], md.y, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[MZ], md.z, CH(task_sample, task_window));
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

  WATCHPOINT(WATCHPOINT_WINDOW);

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

  
  int next_i = (i + 1) % WINDOW_SIZE;
  CHAN_OUT1(int, i, next_i, SELF_OUT_CH(task_window));


  /*Every TEMP_WINDOW_SIZE samples, compute a new average*/
  if( next_i == 0 ){
    TRANSITION_TO(task_update_window_start);
  }else{
    TRANSITION_TO(task_sample);
  }

}

void printsamp(int t, int gx, int gy, int gz, int mx, int my, int mz){
  LOG("{T:%i,G:{%i,%i,%i},M:{%i,%i,%i}}",t,gx,gy,gz,mx,my,mz);
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

  WATCHPOINT(WATCHPOINT_UPDATE_WINDOW_START);

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
    avg[i] = sum[i] >> WINDOW_DIV_SHIFT;
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

 

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_window),
                                                  CH(task_update_proxy,task_update_window));

  /*Update the continuously updated average buffer with this average*/ 

  /* downsample to signed 8-bit int: 7-bit of downsampled data */
#define MAG_DOWNSAMPLE (1 << (12 - 7))  // sensor resolution: 12-bit
#define GYRO_DOWNSAMPLE (1 << (16 - 7)) // sensor resolution: 16-bit

  edb_info.averages[which_window].temp = avg[TEMP] / 10; // to degrees
  edb_info.averages[which_window].gx   = avg[GX] / GYRO_DOWNSAMPLE;
  edb_info.averages[which_window].gy   = avg[GY] / GYRO_DOWNSAMPLE;
  edb_info.averages[which_window].gz   = avg[GZ] / GYRO_DOWNSAMPLE;
  edb_info.averages[which_window].mx   = avg[MX] / MAG_DOWNSAMPLE;
  edb_info.averages[which_window].my   = avg[MY] / MAG_DOWNSAMPLE;
  edb_info.averages[which_window].mz   = avg[MZ] / MAG_DOWNSAMPLE;

  /*Get the index for this window that we need to update*/
  int win_i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_window), 
                                                  CH(task_update_proxy,task_update_window));

  /*Use window ID and win index to self-chan the average, saving it*/
  //WINGET(which_window,win_i,TEMP)
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,TEMP)], avg[TEMP], CH(task_update_window,task_update_proxy));

  CHAN_OUT1(int, windows[WINGET(which_window,win_i,GX)], avg[GX], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,GY)], avg[GY], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,GZ)], avg[GZ], CH(task_update_window,task_update_proxy));
  
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,MX)], avg[MX], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,MY)], avg[MY], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,MZ)], avg[MZ], CH(task_update_window,task_update_proxy));
  /*Send self the next win_i for this window*/
  int next_wini = (win_i + 1) % WINDOW_SIZE;
  CHAN_OUT1(int, win_i[which_window], next_wini, CH(task_update_window,task_update_proxy));

  /*Determine the next window to average*/
  int next_window = (which_window + 1) % NUM_WINDOWS;
  CHAN_OUT1(int, which_window, next_window, CH(task_update_window,task_update_proxy));

    /*For pretty printing*/ 
    /*
    unsigned s; 
    for( s = 0; s < which_window; s++ ){
      LOG(" ");
    }
    LOG("%i[",which_window);
    */
    
    /*Put this average in the next window, 
      then re-average that window*/
    unsigned i;
    for(i = 0; i < WINDOW_SIZE; i++){

      if( i == win_i ){
        /*window[win_i] that goes back to task_update_window_start gets the avg*/
 
        //LOG("W");
        //printsamp(avg[TEMP],avg[GX],avg[GY],avg[GZ],avg[MX],avg[MY],avg[MZ]);

        CHAN_OUT1(int, window[SAMPGET(i,TEMP)], avg[TEMP], CH(task_update_window, task_update_window_start));
        
        CHAN_OUT1(int, window[SAMPGET(i,GX)], avg[GX], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GY)], avg[GY], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GZ)], avg[GZ], CH(task_update_window, task_update_window_start));

        CHAN_OUT1(int, window[SAMPGET(i,MX)], avg[MX], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MY)], avg[MY], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MZ)], avg[MZ], CH(task_update_window, task_update_window_start));


      }else{
        /*window[i != win_i] that goes back to task_update_window_start gets self's window[i]*/

        //LOG("O");
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

        //printsamp(temp,gx,gy,gz,mx,my,mz);

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
    //LOG("]\r\n");

  if(next_window != 0){
    /*Not the last window: average the next one*/
    TRANSITION_TO(task_update_window_start);
  }else{
    /*The last window: go back to sampling*/
    unsigned w;
    for( w = 0; w < NUM_WINDOWS; w++ ){
      printsamp(edb_info.averages[w].temp,
                edb_info.averages[w].gx,
                edb_info.averages[w].gy,
                edb_info.averages[w].gz,
                edb_info.averages[w].mx,
                edb_info.averages[w].my,
                edb_info.averages[w].mz);
      LOG("\r\n");
    }
    LOG("-------\r\n");
    /*Done with all windows!*/
    TRANSITION_TO(task_sample);
  }
}

INIT_FUNC(initializeHardware)
ENTRY_TASK(task_init)
