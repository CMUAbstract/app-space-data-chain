#include <libmsp/mem.h>
#include <libio/log.h>

#include "gyro.h"
#include "mspware/driverlib.h"

const unsigned int gyroBytes = 8;
const unsigned int gyroIdBytes = 1;
volatile unsigned char rawGyroData[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };
volatile unsigned char gyroId = 0;

void gyro_init(void) {


  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, GYRO_SLAVE_ADDRESS);

  EUSCI_B_I2C_enable(EUSCI_B0_BASE);
  
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, GYRO_ID_ADDRESS);
  
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);

  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

  gyroId = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);

  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
  
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

#if 0 // Command to put gyroscope into low-power idle/sleep mode
  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, GYRO_PWRMGT_ADDRESS);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, GYRO_PWRMGT_VALUE);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
#endif

  EUSCI_B_I2C_masterSendStart(EUSCI_B0_BASE);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, GYRO_DLPF_FS_ADDRESS);
  EUSCI_B_I2C_masterSendMultiByteNext(EUSCI_B0_BASE, GYRO_DLPF_FS_VALUE);
  EUSCI_B_I2C_masterSendMultiByteStop(EUSCI_B0_BASE);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));

  EUSCI_B_I2C_disable(EUSCI_B0_BASE);

  LOG("[gyro] Gyro set up. Gyro.WhoAmI: %x\r\n", gyroId);

}

void gyro_read(gyro_t* coordinates) {

  int i;

  /*Set the slave address that we want to read from
  Need to do this here because we might also be reading
  another I2C device on the same bus */
    
  EUSCI_B_I2C_setSlaveAddress(EUSCI_B0_BASE, GYRO_SLAVE_ADDRESS);
  EUSCI_B_I2C_enable(EUSCI_B0_BASE);

  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

  EUSCI_B_I2C_masterSendSingleByte(EUSCI_B0_BASE, GYRO_TEMPH_ADDRESS);
  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  
  EUSCI_B_I2C_setMode(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_MODE);
  EUSCI_B_I2C_masterReceiveStart(EUSCI_B0_BASE);

  for( i = 0; i < gyroBytes; i++ ){
    rawGyroData[i] = EUSCI_B_I2C_masterReceiveSingle(EUSCI_B0_BASE);
  }

  EUSCI_B_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);

  while(EUSCI_B_I2C_isBusBusy(EUSCI_B0_BASE));
  EUSCI_B_I2C_disable(EUSCI_B0_BASE);

  coordinates->x = (rawGyroData[XH] << 8) | rawGyroData[XL];
  coordinates->z = (rawGyroData[YH] << 8) | rawGyroData[YL];
  coordinates->y = (rawGyroData[ZH] << 8) | rawGyroData[ZL];
  LOG("Gyro temp: %i\r\n",(rawGyroData[TEMPH] << 8) | rawGyroData[TEMPL]);

}
