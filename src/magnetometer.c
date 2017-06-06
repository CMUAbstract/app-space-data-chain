#include <libmsp/sleep.h>
#include <libio/console.h>
#include <libmspware/driverlib.h>

#include "magnetometer.h"

#define MAG_ID_LEN 3
#define MAG_SAMPLE_LEN 6

static unsigned char rawMagData[MAG_SAMPLE_LEN];
static unsigned char magnetometerId[MAG_ID_LEN];

bool magnetometer_init(void) {
  int i;

  EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, MAGNETOMETER_SLAVE_ADDRESS);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, MAGNETOMETER_ID_ADDRESS);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  for (i = 0; i < MAG_ID_LEN; i++) {
    if (i == MAG_ID_LEN - 1)
        magnetometerId[i] = EUSCI_B_I2C_masterReceiveMultiByteFinish(EUSCI_B0_BASE);
    else
        magnetometerId[i] = EUSCI_B_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
  }
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  LOG("[mag] chip ID: %c%c%c\r\n",
      magnetometerId[0], magnetometerId[1], magnetometerId[2]);
  if (!(magnetometerId[0] == 'H' && magnetometerId[1] == '4' && magnetometerId[2] == '3')) {
    LOG("[mag] error: invalid chip ID\r\n");
    return false;
  }

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  /* 1 raw sample per data point, normal measurement mode (MS0 MS1 = 00) */
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, MAGNETOMETER_CONFIG_REGISTER_A);
  EUSCI_B_I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, MAGNETOMETER_NUMAVG_1);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  /* Set gain */
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, MAGNETOMETER_CONFIG_REGISTER_B);
  EUSCI_B_I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, MAGNETOMETER_GAIN_1);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_disable(EUSCI_B0_BASE);

  // Wait for analog circuitry to initialize
  msp_sleep(26); // 50ms at ACLK/64=32768/64

  return true;
}

void magnetometer_read(magnet_t* coordinates) {
  int i;

  EUSCI_B_I2C_disable(EUSCI_B0_BASE);
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, MAGNETOMETER_SLAVE_ADDRESS);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, MAGNETOMETER_MODE_REGISTER_ADDRESS);
  EUSCI_B_I2C_masterSendMultiByteFinish(EUSCI_B0_BASE, MAGNETOMETER_MODE_SINGLE_OUTPUT);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  // Wait for sample to be generated
  msp_sleep(4); // ~6ms at ACLK/64=32768/64
  
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);
  for (i = 0; i < MAG_SAMPLE_LEN; i++) {
    if (i == MAG_SAMPLE_LEN - 1)
        rawMagData[i] = EUSCI_B_I2C_masterReceiveMultiByteFinish(EUSCI_B0_BASE);
    else
        rawMagData[i] = EUSCI_B_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
  }
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  EUSCI_B_I2C_disable(EUSCI_B0_BASE);

  LOG2("[mag] raw sample data: ");
  for (i = 0; i < MAG_SAMPLE_LEN; ++i)
      LOG2("%02x ", rawMagData[i]);
  LOG2("\r\n");

  coordinates->x = (rawMagData[0] << 8) | rawMagData[1];
  coordinates->z = (rawMagData[2] << 8) | rawMagData[3];
  coordinates->y = (rawMagData[4] << 8) | rawMagData[5];

  LOG("[mag] sample x %i y %i z %i\r\n",
      coordinates->x, coordinates->y, coordinates->z);
}
