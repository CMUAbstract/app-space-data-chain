#include <msp430.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

#include <libio/console.h>
#include <libchain/chain.h>
#include <libmspware/driverlib.h>
#include <libmsp/watchdog.h>
#include <libmsp/clock.h>
#include <libmsp/gpio.h>
#include <libharvest/charge.h>
#include <libmspuartlink/uartlink.h>

#include "pins.h"
#include "temp_sensor.h"
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

typedef struct _samp_t{
  int temp;
  int gx;
  int gy;
  int gz;
  int mx;
  int my;
  int mz;
} samp_t;

// Type for pkt sent over the radio (via UART)

// Transmit first and last windows only
static const unsigned pkt_window_indexes[] = { 0, NUM_WINDOWS - 1 };
#define PKT_NUM_WINDOWS (sizeof(pkt_window_indexes) / sizeof(pkt_window_indexes[0]))

typedef struct __attribute__((packed)) {
    unsigned temp:8;
    unsigned gx:4;
    unsigned gy:4;
    unsigned gz:4;
    unsigned mx:4;
    unsigned my:4;
    unsigned mz:4;
} pkt_win_t;

typedef struct __attribute__((packed)) {
    pkt_win_t windows[PKT_NUM_WINDOWS];
} pkt_t;

#define GYRO_DOWNSAMPLE_FACTOR 4
#define MAG_DOWNSAMPLE_FACTOR  4

// TODO
/* downsample to signed 8-bit int: 7-bit of downsampled data */
// #define MAG_DOWNSAMPLE (1 << (12 - 7))  // sensor resolution: 12-bit
// #define GYRO_DOWNSAMPLE (1 << (16 - 7)) // sensor resolution: 16-bit


// Channel declarations

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

struct msg_window_averages {
    CHAN_FIELD_ARRAY(samp_t, win_avg, NUM_WINDOWS);
};

struct msg_pkt {
    CHAN_FIELD(int, pkt);
};

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_window)
TASK(4, task_update_window_start)
TASK(5, task_update_window)
TASK(6, task_update_proxy)
TASK(7, task_output)
TASK(8, task_pack)
TASK(9, task_send)

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

MULTICAST_CHANNEL(msg_window_averages, out, task_update_window, task_output, task_pack);
CHANNEL(task_pack, task_send, msg_pkt);

#define WATCHPOINT_BOOT                 0
#define WATCHPOINT_SAMPLE               1
#define WATCHPOINT_WINDOW               2
#define WATCHPOINT_UPDATE_WINDOW_START  3
#define WATCHPOINT_OUTPUT               4

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

static void delay(uint32_t cycles)
{
    unsigned i;
    for (i = 0; i < cycles / (1U << 15); ++i)
        __delay_cycles(1U << 15);
}

void initializeHardware()
{
    msp_watchdog_disable();
    msp_gpio_unlock();

    GPIO(PORT_DBG_0, DIR) |= BIT(PIN_DBG_0);
    GPIO(PORT_DBG_0, OUT) &= ~BIT(PIN_DBG_0);
    GPIO(PORT_DBG_1, DIR) |= BIT(PIN_DBG_1);
    GPIO(PORT_DBG_1, DIR) &= ~BIT(PIN_DBG_1);

    GPIO(PORT_DBG_0, OUT) |= BIT(PIN_DBG_0);

    __enable_interrupt();
    harvest_charge();

    GPIO(PORT_DBG_0, OUT) &= ~BIT(PIN_DBG_0);

    // Set unconnected pins to output low
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

    msp_clock_setup();

#ifdef CONFIG_EDB
    edb_init();
#endif

    WATCHPOINT(WATCHPOINT_BOOT);

    INIT_CONSOLE();

    PRINTF("EDBsat app %u\r\n", 72);

    LOG("i2c init\r\n");
    i2c_setup();

    LOG("mag init\r\n");
    magnetometer_init();

#ifdef ENABLE_GYRO
    LOG("gyro init\r\n");
    gyro_init();
#endif // ENABLE_GYRO

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

  
#ifdef ENABLE_GYRO
  gyro_t gd;
  read_gyro(&(gd.x),&(gd.y),&(gd.z));

  CHAN_OUT1(int, sample[GX], gd.x, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[GY], gd.y, CH(task_sample, task_window));
  CHAN_OUT1(int, sample[GZ], gd.z, CH(task_sample, task_window));
#endif // ENABLE_GYRO



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

#ifdef ENABLE_GYRO
  int gx = *CHAN_IN1(int, sample[GX], CH(task_sample, task_window));
  int gy = *CHAN_IN1(int, sample[GY], CH(task_sample, task_window));
  int gz = *CHAN_IN1(int, sample[GZ], CH(task_sample, task_window));
#endif // ENABLE_GYRO

  int mx = *CHAN_IN1(int, sample[MX], CH(task_sample, task_window));

  int my = *CHAN_IN1(int, sample[MY], CH(task_sample, task_window));

  int mz = *CHAN_IN1(int, sample[MZ], CH(task_sample, task_window));
  
  
  CHAN_OUT1(int, window[SAMPGET(i,TEMP)], temp, CH(task_window, task_update_window_start));
#ifdef ENABLE_GYRO
  CHAN_OUT1(int, window[SAMPGET(i,GX)], gx, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,GY)], gy, CH(task_window, task_update_window_start));
  CHAN_OUT1(int, window[SAMPGET(i,GZ)], gz, CH(task_window, task_update_window_start));
#endif // ENABLE_GYRO
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
#ifdef ENABLE_GYRO
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
#endif // ENABLE_GYRO
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

#ifdef ENABLE_GYRO
  int gx = *CHAN_IN2(int, windows[WINGET(which_window,i,GX)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int gy = *CHAN_IN2(int, windows[WINGET(which_window,i,GY)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int gz = *CHAN_IN2(int, windows[WINGET(which_window,i,GZ)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
#endif // ENABLE_GYRO

  int mx = *CHAN_IN2(int, windows[WINGET(which_window,i,MX)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int my = *CHAN_IN2(int, windows[WINGET(which_window,i,MY)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));
  int mz = *CHAN_IN2(int, windows[WINGET(which_window,i,MZ)], CH(task_init,task_update_proxy),
                                                              CH(task_update_window,task_update_proxy));


  CHAN_OUT1(int, which_window, which_window, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, win_i[which_window], i, CH(task_update_proxy,task_update_window));
  
  CHAN_OUT1(int, windows[WINGET(which_window,i,TEMP)], temp, CH(task_update_proxy,task_update_window));
#ifdef ENABLE_GYRO
  CHAN_OUT1(int, windows[WINGET(which_window,i,GX)], gx, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,GY)], gy, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,GZ)], gz, CH(task_update_proxy,task_update_window));
#endif // ENABLE_GYRO
  CHAN_OUT1(int, windows[WINGET(which_window,i,MX)], mx, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,MY)], my, CH(task_update_proxy,task_update_window));
  CHAN_OUT1(int, windows[WINGET(which_window,i,MZ)], mz, CH(task_update_proxy,task_update_window));

  TRANSITION_TO(task_update_window);

}

void task_update_window(){

  /*Get the average and window ID from the averaging call*/
  int avg[SAMPLE_SIZE];
  avg[TEMP] = *CHAN_IN1(int, average[TEMP], CH(task_update_window_start, task_update_window));
#ifdef ENABLE_GYRO
  avg[GX] = *CHAN_IN1(int, average[GX], CH(task_update_window_start, task_update_window));
  avg[GY] = *CHAN_IN1(int, average[GY], CH(task_update_window_start, task_update_window));
  avg[GZ] = *CHAN_IN1(int, average[GZ], CH(task_update_window_start, task_update_window));
#endif // ENABLE_GYRO
  avg[MX] = *CHAN_IN1(int, average[MX], CH(task_update_window_start, task_update_window));
  avg[MY] = *CHAN_IN1(int, average[MY], CH(task_update_window_start, task_update_window));
  avg[MZ] = *CHAN_IN1(int, average[MZ], CH(task_update_window_start, task_update_window));

 

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_window),
                                                  CH(task_update_proxy,task_update_window));

  /*Get the index for this window that we need to update*/
  int win_i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_window), 
                                                  CH(task_update_proxy,task_update_window));

  /* Window average is ready for this window, forward to output, packing, and sending tasks */
  samp_t win_avg = {
      .temp = avg[TEMP],
      .gx = avg[GX],
      .gy = avg[GY],
      .gz = avg[GZ],
      .mx = avg[MX],
      .my = avg[MY],
      .mz = avg[MZ]
  };
  PRINTF("SEND {T:%i, G:{%i,%i,%i},M:{%i,%i,%i}}\r\n",
      win_avg.temp,
      win_avg.gx, win_avg.gy, win_avg.gz,
      win_avg.mx, win_avg.my, win_avg.mz);

  CHAN_OUT1(samp_t, win_avg[which_window], win_avg,
            MC_OUT_CH(out, task_update_window, task_output, task_pack));

  /*Use window ID and win index to self-chan the average, saving it*/
  //WINGET(which_window,win_i,TEMP)
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,TEMP)], avg[TEMP], CH(task_update_window,task_update_proxy));

#ifdef ENABLE_GYRO
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,GX)], avg[GX], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,GY)], avg[GY], CH(task_update_window,task_update_proxy));
  CHAN_OUT1(int, windows[WINGET(which_window,win_i,GZ)], avg[GZ], CH(task_update_window,task_update_proxy));
#endif // ENABLE_GYRO
  
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
        
#ifdef ENABLE_GYRO
        CHAN_OUT1(int, window[SAMPGET(i,GX)], avg[GX], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GY)], avg[GY], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GZ)], avg[GZ], CH(task_update_window, task_update_window_start));
#endif // ENABLE_GYRO

        CHAN_OUT1(int, window[SAMPGET(i,MX)], avg[MX], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MY)], avg[MY], CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,MZ)], avg[MZ], CH(task_update_window, task_update_window_start));


      }else{
        /*window[i != win_i] that goes back to task_update_window_start gets self's window[i]*/

        //LOG("O");
        /*In the value from the self channel*/
        int temp = *CHAN_IN2(int, windows[WINGET(which_window,i,TEMP)], CH(task_init,task_update_window),
                                                                        CH(task_update_proxy,task_update_window));

#ifdef ENABLE_GYRO
        int gx = *CHAN_IN2(int, windows[WINGET(which_window,i,GX)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int gy = *CHAN_IN2(int, windows[WINGET(which_window,i,GY)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int gz = *CHAN_IN2(int, windows[WINGET(which_window,i,GZ)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
#endif // ENABLE_GYRO

        int mx = *CHAN_IN2(int, windows[WINGET(which_window,i,MX)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int my = *CHAN_IN2(int, windows[WINGET(which_window,i,MY)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));
        int mz = *CHAN_IN2(int, windows[WINGET(which_window,i,MZ)], CH(task_init,task_update_window),
                                                                    CH(task_update_proxy,task_update_window));

        //printsamp(temp,gx,gy,gz,mx,my,mz);

        /*Out the value back to task_update_window_start for averaging*/
        CHAN_OUT1(int, window[SAMPGET(i,TEMP)], temp, CH(task_update_window, task_update_window_start));
        
#ifdef ENABLE_GYRO
        CHAN_OUT1(int, window[SAMPGET(i,GX)], gx, CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GY)], gy, CH(task_update_window, task_update_window_start));
        CHAN_OUT1(int, window[SAMPGET(i,GZ)], gz, CH(task_update_window, task_update_window_start));
#endif // ENABLE_GYRO

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
    /*The last window: output, then go back to sampling*/
    TRANSITION_TO(task_output);
  }
}

void task_output() {
    for( unsigned w = 0; w < NUM_WINDOWS; w++ ){
      samp_t win_avg = *CHAN_IN1(samp_t, win_avg[w], MC_IN_CH(out, task_update_window, task_output));
      PRINTF("OUT %u {T:%i,G:{%i,%i,%i},M:{%i,%i,%i}}\r\n",
          w, win_avg.temp,
          win_avg.gx, win_avg.gy, win_avg.gz,
          win_avg.mx, win_avg.my, win_avg.mz);
    }
    TRANSITION_TO(task_pack);
}

void task_pack() {
    pkt_t pkt;

    for( unsigned i = 0; i < PKT_NUM_WINDOWS; i++ ){
      unsigned w = pkt_window_indexes[i];

      samp_t win_avg = *CHAN_IN1(samp_t, win_avg[w], MC_IN_CH(out, task_update_window, task_output));
      pkt.windows[w].temp = win_avg.temp; // use full byte

#ifdef ENABLE_GYRO
      pkt.windows[w].gx = win_avg.gx / GYRO_DOWNSAMPLE_FACTOR;
      pkt.windows[w].gy = win_avg.gy / GYRO_DOWNSAMPLE_FACTOR;
      pkt.windows[w].gz = win_avg.gz / GYRO_DOWNSAMPLE_FACTOR;
#endif // ENABLE_GYRO

      pkt.windows[w].mx = win_avg.mx / MAG_DOWNSAMPLE_FACTOR;
      pkt.windows[w].my = win_avg.my / MAG_DOWNSAMPLE_FACTOR;
      pkt.windows[w].mz = win_avg.mz / MAG_DOWNSAMPLE_FACTOR;
    }
    CHAN_OUT1(pkt_t, pkt, pkt, CH(task_pack, task_send));
    TRANSITION_TO(task_send);
}

void task_send() {
    WATCHPOINT(WATCHPOINT_OUTPUT);

    pkt_t pkt = *CHAN_IN1(pkt_t, pkt, CH(task_pack, task_send));

    LOG("pkt (len %u): ", sizeof(pkt_t));
    for (unsigned i = 0; i < sizeof(pkt_t); ++i) {
        LOG("%02x ", *(((uint8_t *)&pkt) + i));
    }
    LOG("\r\n");

    uartlink_open_tx();
    uartlink_send((uint8_t *)&pkt, sizeof(pkt_t));
    uartlink_close();

    /* Loop back to the beginning */
    TRANSITION_TO(task_sample);
}

INIT_FUNC(initializeHardware)
ENTRY_TASK(task_init)
