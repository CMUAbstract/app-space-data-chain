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
#include "lsm.h"

// Must be after any header that includes mps430.h due to
// the workround of undef'ing 'OUT' (see pin_assign.h)
#ifdef CONFIG_EDB
#include <libedb/edb.h>
#else
#define WATCHPOINT(...)
#endif

#define WINDOW_SIZE 4 /*number of samples in a window*/
#define NUM_WINDOWS 4
#define WINDOW_DIV_SHIFT 2 /* 2^WINDOW_DIV_SHIFT = NUM_WINDOWS */
#define WINDOWS_SIZE 16 /* NUM_WINDOWS * WINDOW_SIZE (libchain needs a literal number) */

/*Get coordinate coor from the sample samp in window win -- windows[WINGET(0,1)*/
#define WINGET(win,samp) (WINDOW_SIZE*win + samp)

typedef struct _samp_t{
  int temp;
  int mx;
  int my;
  int mz;
  int ax;
  int ay;
  int az;
  int gx;
  int gy;
  int gz;
} samp_t;

// Type for pkt sent over the radio (via UART)

// Transmit first and last windows only
static const unsigned pkt_window_indexes[] = { 0, NUM_WINDOWS - 1 };
#define PKT_NUM_WINDOWS (sizeof(pkt_window_indexes) / sizeof(pkt_window_indexes[0]))

typedef struct __attribute__((packed)) {
    int temp:8;
    int mx:4;
    int my:4;
    int mz:4;
    int ax:4;
    int ay:4;
    int az:4;
    int gx:4;
    int gy:4;
    int gz:4;
} pkt_win_t;

typedef struct __attribute__((packed)) {
    pkt_win_t windows[PKT_NUM_WINDOWS];
} pkt_t;

/* downsample to signed 4-bit int: 4-bit of downsampled data (i.e [-2^3,+2^3])*/
#define PKT_FIELD_MAG_BITS 4
#define SENSOR_BITS_MAG    12
#define NOT_FULL_SCALE_FACTOR_MAG 2 /* n, where scaling factor is decreased by 2^n because
                                       the actual dynamic range of the quantity is smaller than full scale */
#define MAG_DOWNSAMPLE_FACTOR (1 << (SENSOR_BITS_MAG - PKT_FIELD_MAG_BITS - NOT_FULL_SCALE_FACTOR_MAG))


#define PKT_FIELD_ACCEL_BITS 4
#define SENSOR_BITS_ACCEL    16
#define NOT_FULL_SCALE_FACTOR_ACCEL 0 /* n, where scaling factor is decreased by 2^n because
                                       the actual dynamic range of the quantity is smaller than full scale */
#define ACCEL_DOWNSAMPLE_FACTOR (1 << (SENSOR_BITS_ACCEL - PKT_FIELD_ACCEL_BITS - NOT_FULL_SCALE_FACTOR_ACCEL))

#define ACCEL_MIN  (-(1 << (PKT_FIELD_ACCEL_BITS - 1))) // -1 because signed
#define ACCEL_MAX  ((1 << (PKT_FIELD_ACCEL_BITS - 1)) - 1) // -1 because signed, -1 because max value


#define PKT_FIELD_GYRO_BITS 4
#define SENSOR_BITS_GYRO    16
#define NOT_FULL_SCALE_FACTOR_GYRO 1 /* n, where scaling factor is decreased by 2^n because
                                       the actual dynamic range of the quantity is smaller than full scale */
#define GYRO_DOWNSAMPLE_FACTOR (1 << (SENSOR_BITS_GYRO - PKT_FIELD_GYRO_BITS - NOT_FULL_SCALE_FACTOR_GYRO))

#define GYRO_MIN  (-(1 << (PKT_FIELD_GYRO_BITS - 1))) // -1 because signed
#define GYRO_MAX  ((1 << (PKT_FIELD_GYRO_BITS - 1)) - 1) // -1 because signed, -1 because max value


static bool mag_ok;
static bool lsm_ok;

// Put it here instead of on the stack
static lsm_t lsm_samp;

// Channel declarations

struct msg_sample{
    CHAN_FIELD(samp_t, sample);
};

struct msg_sample_avg_in{
  CHAN_FIELD(task_t *, next_task);
  CHAN_FIELD_ARRAY(samp_t, window, WINDOW_SIZE);
};

struct msg_sample_avg_out{
  CHAN_FIELD(samp_t, average);
};

struct msg_sample_window{
  CHAN_FIELD_ARRAY(samp_t, window, WINDOW_SIZE);
};


struct msg_sample_windows{
    CHAN_FIELD(int, which_window);
    CHAN_FIELD_ARRAY(int, win_i, WINDOW_SIZE);
    CHAN_FIELD_ARRAY(samp_t, windows, NUM_WINDOWS * WINDOW_SIZE);
};

struct msg_self_sample_windows{
    SELF_CHAN_FIELD(int, which_window);
    SELF_CHAN_FIELD_ARRAY(int, win_i, WINDOW_SIZE);
    SELF_CHAN_FIELD_ARRAY(samp_t, windows, WINDOWS_SIZE);
};
#define FIELD_INIT_msg_self_sample_windows { \
    SELF_FIELD_INITIALIZER, \
    SELF_FIELD_ARRAY_INITIALIZER(WINDOW_SIZE), \
    SELF_FIELD_ARRAY_INITIALIZER(WINDOWS_SIZE) \
}

struct msg_index{
    CHAN_FIELD(int, i);
    CHAN_FIELD(bool, first_window);
};

struct msg_self_index{
    SELF_CHAN_FIELD(int, i);
    SELF_CHAN_FIELD(bool, first_window);
};
#define FIELD_INIT_msg_self_index { \
    SELF_FIELD_INITIALIZER \
}

struct msg_window_averages {
    CHAN_FIELD_ARRAY(samp_t, win_avg, NUM_WINDOWS);
};

struct msg_pkt {
    CHAN_FIELD(pkt_t, pkt);
};

TASK(1, task_init)
TASK(2, task_sample)
TASK(3, task_window)
TASK(4, task_update_window_start)
TASK(5, task_update_window)
TASK(6, task_output)
TASK(7, task_pack)
TASK(8, task_send)

/*Channels to window*/
CHANNEL(task_sample, task_window, msg_sample);

CHANNEL(task_init, task_window, msg_index);
SELF_CHANNEL(task_window, msg_self_index);
CHANNEL(task_window, task_update_window, msg_sample_windows);

/*Window channels to update_window_start*/
CHANNEL(task_init, task_update_window_start, msg_sample_window);
CHANNEL(task_window, task_update_window_start, msg_sample_window);
CHANNEL(task_update_window, task_update_window_start, msg_sample_window);
CHANNEL(task_update_window_start, task_update_window, msg_sample_avg_out);

/*Windows channels to task_update_window*/
SELF_CHANNEL(task_update_window, msg_self_sample_windows);
CHANNEL(task_init, task_update_window, msg_sample_windows);

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
  param.dataRate = EUSCI_B_I2C_SET_DATA_RATE_100KBPS;
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

    LOG("EDBsat app\r\n");

    LOG("i2c init\r\n");
    i2c_setup();

    LOG("mag init\r\n");
    mag_ok = magnetometer_init();

    LOG("LSM init\r\n");
    lsm_ok = lsm_init();

    LOG("space app: curtsk %u\r\n", curctx->task->idx);
}

void print_sample(samp_t *s) {
  LOG("{T:%i,"
      "M:{%i,%i,%i},"
      "A:{%i,%i,%i},"
      "G:{%i,%i,%i}"
      "}\r\n",
      s->temp,
      s->mx,s->my,s->mz,
      s->ax, s->ay, s->az,
      s->gx, s->gy, s->gz);
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
    bool vtrue = true;
    for( i = 0; i < NUM_WINDOWS; i++ ){
      /*Zero every window's win_i*/
      CHAN_OUT1(int, win_i[i], zero, CH(task_init, task_update_window));
    }

    CHAN_OUT1(int, which_window, zero, CH(task_init, task_update_window));
    CHAN_OUT1(int, i, zero, CH(task_init, task_window));
    CHAN_OUT1(int, first_window, vtrue, CH(task_init, task_window));

    TRANSITION_TO(task_sample);
}

void read_mag(int *x,
              int *y,
              int *z){
  magnet_t co;
  if (mag_ok) {
    magnetometer_read(&co);
    *x = co.x;
    *y = co.y;
    *z = co.z;
  } else {
    *x = 0;
    *y = 0;
    *z = 0;
  }
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
  
  LOG("task sample\r\n");

  WATCHPOINT(WATCHPOINT_SAMPLE);

  samp_t sample;
  sample.temp = read_temperature_sensor();

  read_mag(&(sample.mx),&(sample.my),&(sample.mz));

  lsm_sample(&lsm_samp);
  sample.ax = lsm_samp.ax;
  sample.ay = lsm_samp.ay;
  sample.az = lsm_samp.az;
  sample.gx = lsm_samp.gx;
  sample.gy = lsm_samp.gy;
  sample.gz = lsm_samp.gz;
  
  CHAN_OUT1(samp_t, sample, sample, CH(task_sample, task_window));
  LOG("sampled: "); print_sample(&sample);

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

  LOG("task window\r\n");

  WATCHPOINT(WATCHPOINT_WINDOW);

  int i = *CHAN_IN2(int, i, SELF_IN_CH(task_window),
                            CH(task_init, task_window));

  samp_t sample = *CHAN_IN1(samp_t, sample, CH(task_sample, task_window));
  CHAN_OUT1(samp_t, window[i], sample, CH(task_window, task_update_window_start));
  
  int next_i = (i + 1) % WINDOW_SIZE;
  CHAN_OUT1(int, i, next_i, SELF_OUT_CH(task_window));

  /*Every TEMP_WINDOW_SIZE samples, compute a new average*/
  if( next_i == 0 ){

    // Initialize all windows in the cascade with th first sample
    // The windows start with the same values, but will change at different "rates"
    bool first_window = *CHAN_IN2(bool, first_window, CH(task_init, task_window),
                                                      SELF_IN_CH(task_window));
    LOG("first window: %u\r\n", first_window);
    if (first_window) {
      for (unsigned which_window = 0; which_window < NUM_WINDOWS; ++which_window) {
          for( i = 0; i < WINDOW_SIZE; i++ ){
            CHAN_OUT1(samp_t, windows[WINGET(which_window,i)], sample, CH(task_window, task_update_window));
          }
      }
      first_window = !first_window;
      CHAN_OUT1(bool, first_window, first_window, SELF_OUT_CH(task_window));
    }

    TRANSITION_TO(task_update_window_start);
  }else{
    TRANSITION_TO(task_sample);
  }

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

  LOG("task update_window_start\r\n");

  WATCHPOINT(WATCHPOINT_UPDATE_WINDOW_START);

  // not samp_t because need 32-bit to prevent overflow
  long sum_temp = 0;
  long sum_mx = 0, sum_my = 0, sum_mz = 0;
  long sum_ax = 0, sum_ay = 0, sum_az = 0;
  long sum_gx = 0, sum_gy = 0, sum_gz = 0;

  samp_t avg;

  for(unsigned j = 0; j < WINDOW_SIZE; j++){

      samp_t sample = *CHAN_IN2(samp_t, window[j], CH(task_window, task_update_window_start),
                                                   CH(task_update_window, task_update_window_start));
      sum_temp += sample.temp;

      sum_mx += sample.mx;
      sum_my += sample.my;
      sum_mz += sample.mz;

      sum_ax += sample.ax;
      sum_ay += sample.ay;
      sum_az += sample.az;

      sum_gx += sample.gx;
      sum_gy += sample.gy;
      sum_gz += sample.gz;
  }
  LOG("sum done\r\n");

  avg.temp = sum_temp / WINDOW_SIZE;
  avg.mx = sum_mx / WINDOW_SIZE;
  avg.my = sum_my / WINDOW_SIZE;
  avg.mz = sum_mz / WINDOW_SIZE;

  avg.ax = sum_ax / WINDOW_SIZE;
  avg.ay = sum_ay / WINDOW_SIZE;
  avg.az = sum_az / WINDOW_SIZE;

  avg.gx = sum_gx / WINDOW_SIZE;
  avg.gy = sum_gy / WINDOW_SIZE;
  avg.gz = sum_gz / WINDOW_SIZE;

  LOG("avg: "); print_sample(&avg);

  CHAN_OUT1(samp_t, average, avg, CH(task_update_window_start,task_update_window));

  TRANSITION_TO(task_update_window);
}

void task_update_window(){

  LOG("task update_window\r\n");

  /*Get the average and window ID from the averaging call*/
  samp_t avg = *CHAN_IN1(samp_t, average, CH(task_update_window_start, task_update_window));

  int which_window = *CHAN_IN2(int, which_window, CH(task_init,task_update_window),
                                                  SELF_IN_CH(task_update_window));

  /*Get the index for this window that we need to update*/
  int win_i = *CHAN_IN2(int, win_i[which_window], CH(task_init,task_update_window), 
                                                  SELF_IN_CH(task_update_window));

  /* Window average is ready for this window, forward to output, packing, and sending tasks */
  LOG("SEND {T:%i,"
         "M:{%i,%i,%i},"
         "A:{%i,%i,%i},"
         "G:{%i,%i,%i}\r\n",
      avg.temp,
      avg.ax, avg.ay, avg.az,
      avg.gx, avg.gy, avg.gz,
      avg.mx, avg.my, avg.mz);

  CHAN_OUT1(samp_t, win_avg[which_window], avg,
            MC_OUT_CH(out, task_update_window, task_output, task_pack));

  /*Use window ID and win index to self-chan the average, saving it*/
  //WINGET(which_window,win_i,TEMP)
  CHAN_OUT1(samp_t, windows[WINGET(which_window,win_i)], avg, SELF_OUT_CH(task_update_window));

  /*Send self the next win_i for this window*/
  int next_wini = (win_i + 1) % WINDOW_SIZE;
  CHAN_OUT1(int, win_i[which_window], next_wini, SELF_OUT_CH(task_update_window));

  /*Determine the next window to average*/
  int next_window = (which_window + 1) % NUM_WINDOWS;
  CHAN_OUT1(int, which_window, next_window, SELF_OUT_CH(task_update_window));

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
        CHAN_OUT1(samp_t, window[i], avg, CH(task_update_window, task_update_window_start));
        
      }else{
        /*window[i != win_i] that goes back to task_update_window_start gets self's window[i]*/

        //LOG("O");
        /*In the value from the self channel*/
        samp_t sample = *CHAN_IN2(samp_t, windows[WINGET(which_window,i)], CH(task_window,task_update_window),
                                                                           SELF_IN_CH(task_update_window));

        /*Out the value back to task_update_window_start for averaging*/
        CHAN_OUT1(samp_t, window[i], sample, CH(task_update_window, task_update_window_start));
      }

    }
    //LOG("]\r\n");

  if(next_window != 0){
    /*Not the last window: average the next one*/
    TRANSITION_TO(task_update_window_start);
  }else{
    /*The last window: output, then go back to sampling*/
#if VERBOSE > 0
    TRANSITION_TO(task_output);
#else // VERBOSE
    TRANSITION_TO(task_pack);
#endif // VERBOSE
  }
}

void task_output() {
#if VERBOSE > 0
  LOG("task output\r\n");
    for( unsigned w = 0; w < NUM_WINDOWS; w++ ){
      samp_t win_avg = *CHAN_IN1(samp_t, win_avg[w], MC_IN_CH(out, task_update_window, task_output));
      LOG("OUT %u {T:%03i,"
          "M:{%05i,%05i,%05i}},"
          "A:{%05i,%05i,%05i}},"
          "G:{%05i,%05i,%05i}\r\n",
          w, win_avg.temp,
          win_avg.mx, win_avg.my, win_avg.mz,
          win_avg.ax, win_avg.ay, win_avg.az,
          win_avg.gx, win_avg.gy, win_avg.gz);
    }
#endif // VERBOSE
    TRANSITION_TO(task_pack);
}

static int scale_mag_sample(int v, int neg_edge, int pos_edge, int overflow)
{
  int scaled;
  if (v == -4096) {
      scaled = overflow;
  } else {
    scaled = v / MAG_DOWNSAMPLE_FACTOR;
    if (scaled < neg_edge) {
      scaled = neg_edge;
    } else if (scaled > pos_edge) {
      scaled = pos_edge;
    }
  }
  return scaled;
}

static int scale_lsm_sample(int v, int factor, int min, int max)
{
  v /= factor;
  if (v < min)
    v = min;
  if (v > max)
    v = max;
  return v;
}

void task_pack() {

    LOG("task pack\r\n");

    pkt_t pkt;

    for( unsigned i = 0; i < PKT_NUM_WINDOWS; i++ ){
      unsigned w = pkt_window_indexes[i];

      samp_t win_avg = *CHAN_IN1(samp_t, win_avg[w], MC_IN_CH(out, task_update_window, task_output));

      LOG("packing: win %u {T:%03i,"
          "M:{%05i,%05i,%05i}}"
          "A:{%05i,%05i,%05i},"
          "G:{%05i,%05i,%05i}\r\n",
          w, win_avg.temp,
          win_avg.mx, win_avg.my, win_avg.mz,
          win_avg.ax, win_avg.ay, win_avg.az,
          win_avg.gx, win_avg.gy, win_avg.gz);

      pkt.windows[w].temp = win_avg.temp; // use full byte



      // Normally, sensor returns a value in [-2048, 2047].
      // On either overflow, sensor return -4096.
      //
      // We shrink the valid range to [-2047,2047] and
      // reserve -2048 for overflow.
      //
      // Then, we also truncate.
      int abs_edge = 1 << (PKT_FIELD_MAG_BITS - 1); // -1, ie. div by 2, because signed
      int neg_edge = -(abs_edge - 1); // reserve for overflow
      int pos_edge = abs_edge - 1;
      int overflow = -abs_edge;
      LOG("scaling: abs %i [%i, %i] ovflw %i\r\n", abs_edge, neg_edge, pos_edge, overflow);

      pkt.windows[w].mx = scale_mag_sample(win_avg.mx, neg_edge, pos_edge, overflow);
      pkt.windows[w].my = scale_mag_sample(win_avg.my, neg_edge, pos_edge, overflow);
      pkt.windows[w].mz = scale_mag_sample(win_avg.mz, neg_edge, pos_edge, overflow);

      // Accel and gyro are simple (since there's no special overflow value)
      pkt.windows[w].ax = scale_lsm_sample(win_avg.ax, ACCEL_DOWNSAMPLE_FACTOR, ACCEL_MIN, ACCEL_MAX);
      pkt.windows[w].ay = scale_lsm_sample(win_avg.ay, ACCEL_DOWNSAMPLE_FACTOR, ACCEL_MIN, ACCEL_MAX);
      pkt.windows[w].az = scale_lsm_sample(win_avg.az, ACCEL_DOWNSAMPLE_FACTOR, ACCEL_MIN, ACCEL_MAX);
      pkt.windows[w].gx = scale_lsm_sample(win_avg.gx, GYRO_DOWNSAMPLE_FACTOR, GYRO_MIN, GYRO_MAX);
      pkt.windows[w].gy = scale_lsm_sample(win_avg.gy, GYRO_DOWNSAMPLE_FACTOR, GYRO_MIN, GYRO_MAX);
      pkt.windows[w].gz = scale_lsm_sample(win_avg.gz, GYRO_DOWNSAMPLE_FACTOR, GYRO_MIN, GYRO_MAX);

      LOG("scaled (/ %u): t %i | mx %i my %i mz %i | ax %i ay %i az %i | gx %i gy %i gz %i\r\n",
           MAG_DOWNSAMPLE_FACTOR,
           pkt.windows[w].temp,
           pkt.windows[w].mx, pkt.windows[w].my, pkt.windows[w].mz,
           pkt.windows[w].ax, pkt.windows[w].ay, pkt.windows[w].az,
           pkt.windows[w].gx, pkt.windows[w].gy, pkt.windows[w].gz);
    }

    LOG("unpacked: t %i | mx %i my %i mz %i | ax %i ay %i az %i | gx %i gy %i gz %i\r\n",
        (int)pkt.windows[0].temp,
        (int)pkt.windows[0].mx * MAG_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].my * MAG_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].mz * MAG_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].ax * ACCEL_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].ay * ACCEL_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].az * ACCEL_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].gx * GYRO_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].gy * GYRO_DOWNSAMPLE_FACTOR,
        (int)pkt.windows[0].gz * GYRO_DOWNSAMPLE_FACTOR);

    CHAN_OUT1(pkt_t, pkt, pkt, CH(task_pack, task_send));
    TRANSITION_TO(task_send);
}

void task_send() {
  LOG("task send\r\n");

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
