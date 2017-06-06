#include <libmspware/driverlib.h>
#include <libio/console.h>

#include "lsm.h"

#define LSM_SLAVE_ADDRESS 0x6b /* 1101011 */

#define LSM_REG_WHO_AM_I 0x0F

#define LSM_WHO_AM_I 0x69

bool lsm_init()
{
  UCB0CTLW0 &= ~UCSWRST; // enable
  UCB0I2CSA = LSM_SLAVE_ADDRESS;

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

  return true;
}
