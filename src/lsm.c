#include <libmspware/driverlib.h>
#include <libio/console.h>
#include <libmsp/sleep.h>

#include "lsm.h"

#define LSM_SLAVE_ADDRESS 0x6b /* 1101011 */

#define LSM_REG_WHO_AM_I 0x0F
#define LSM_REG_CTRL1_XL 0x10
#define LSM_REG_CTRL2_G  0x11
#define LSM_REG_OUTX_L_XL 0x28
#define LSM_REG_OUTX_L_G 0x22

#define LSM_ODR_XL_12_5_HZ  0x10
#define LSM_ODR_XL_52_HZ    0x30

#define LSM_ODR_G_52_HZ    0x30
#define LSM_FS_125         0x02 /* minimum */

#define LSM_WHO_AM_I 0x69

#define SAMPLE_PERIOD 10 /* @ 52Hz (must match ODR setting): ~20ms in ACLK/64 */
#define SAMPLE_LEN 12

static uint8_t sample_bytes[SAMPLE_LEN];


static void set_reg(unsigned reg, unsigned val)
{
  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start

  // Have to wait for addr transmission to finish, otherwise the TXIFG does not
  // behave as expected (both reg and val writes below fall through, despite
  // waits on TXIFG).
  while((UCB0CTLW0 & UCTXSTT));

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = reg;

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = val;

  while(!(UCB0IFG & UCTXIFG));
  UCB0CTLW0 |= UCTXSTP; // stop

  while (UCB0STATW & UCBBUSY);
}

bool lsm_init()
{
  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = LSM_SLAVE_ADDRESS;
  UCB0CTLW0 &= ~UCSWRST; // enable

  while (UCB0STATW & UCBBUSY);

  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = LSM_REG_WHO_AM_I;
  while(!(UCB0IFG & UCTXIFG));

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  // wait for addr transmission to finish, data transfer to start
  while(UCB0CTLW0 & UCTXSTT);
  UCB0CTLW0 |= UCTXSTP; // stop

  while(!(UCB0IFG & UCRXIFG));
  uint8_t id = UCB0RXBUF;

  while (UCB0STATW & UCBBUSY);

  if (id != LSM_WHO_AM_I) {
    LOG("invalid LSM id: 0x%02x (expected 0x%02x)\r\n", id, LSM_WHO_AM_I);
    return false;
  }

  LOG("LSM id: 0x%02x\r\n", id);

  set_reg(LSM_REG_CTRL1_XL, LSM_ODR_XL_52_HZ);
  set_reg(LSM_REG_CTRL2_G, LSM_ODR_G_52_HZ | LSM_FS_125);

  return true;
}

void lsm_sample(lsm_t *sample) {

  // Wait for the first sample @ 52Hz
  //
  // NOTE: yeah, this does not need to be blocking, but we can't
  // be sure that the app won't finish executing an interation
  // before this sensor period elapses period. 
  msp_sleep(SAMPLE_PERIOD); /* ~20ms @ ACLK/64 */

  UCB0CTLW0 |= UCSWRST; // disable
  UCB0I2CSA = LSM_SLAVE_ADDRESS;
  UCB0CTLW0 &= ~UCSWRST; // enable

  UCB0CTLW0 |= UCTR | UCTXSTT; // transmit mode and start
  while((UCB0CTLW0 & UCTXSTT));

  while(!(UCB0IFG & UCTXIFG));
  UCB0TXBUF = LSM_REG_OUTX_L_G;
  while(!(UCB0IFG & UCTXIFG));

  UCB0CTLW0 &= ~UCTR; // receive mode
  UCB0CTLW0 |= UCTXSTT; // repeated start

  while(UCB0CTLW0 & UCTXSTT);

  for (unsigned i = 0; i < SAMPLE_LEN; ++i) {
    if (i == SAMPLE_LEN - 1)
        UCB0CTLW0 |= UCTXSTP; // stop

    while(!(UCB0IFG & UCRXIFG));
    sample_bytes[i] = UCB0RXBUF;
  }

  while (UCB0STATW & UCBBUSY);

  LOG2("[lsm] sample bytes: ");
  for (unsigned i = 0; i < SAMPLE_LEN; ++i) {
    LOG2("%02x ", sample_bytes[i]);
  }
  LOG2("\r\n");

  sample->gx = ( sample_bytes[1] << 8) | sample_bytes[0];
  sample->gy = ( sample_bytes[3] << 8) | sample_bytes[2];
  sample->gz = ( sample_bytes[5] << 8) | sample_bytes[4];
  sample->ax = ( sample_bytes[7] << 8) | sample_bytes[6];
  sample->ay = ( sample_bytes[9] << 8) | sample_bytes[8];
  sample->az = (sample_bytes[11] << 8) | sample_bytes[10];

  LOG("[lsm] sample: ax %i ay %i az %i gx %i gy %i gz %i\r\n",
      sample->ax, sample->ay, sample->az,
      sample->gx, sample->gy, sample->gz);

}
