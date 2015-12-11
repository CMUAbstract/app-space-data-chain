#ifndef GYRO_H__
#define GYRO_H__

#define GYRO_SLAVE_ADDRESS 0x68 /*from datasheet: 1101000 */
                                /*note: datasheet sez 110100x - x is 0 or 1 to have 2 gyros*/

/*Need to i2c write this in gyro_init*/
#define GYRO_DLPF_FS_ADDRESS 0x16 /*"full-scale" config register*/
#define GYRO_DLPF_FS_VALUE 0x18 /*magic value for "full-scale" meas., req'd to run*/

/*Data registers*/
#define GYRO_TEMPH_ADDRESS 0x1B
#define GYRO_TEMPL_ADDRESS 0x1C
#define GYRO_XH_ADDRESS 0x1D
#define GYRO_XL_ADDRESS 0x1E
#define GYRO_YH_ADDRESS 0x1F
#define GYRO_YL_ADDRESS 0x20
#define GYRO_ZH_ADDRESS 0x21
#define GYRO_ZL_ADDRESS 0x22

#define XH 0
#define XL 1
#define YH 2
#define YL 3
#define ZH 4
#define ZL 5

typedef struct {
  int x;
  int y;
  int z;
} gyro_t;

void gyro_init(void);
void gyro_read(gyro_t* coordinates);

#endif /*GYRO_H__*/
