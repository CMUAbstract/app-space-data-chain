#ifndef MAGNETOMETER_H__
#define MAGNETOMETER_H__

#define MAGNETOMETER_SLAVE_ADDRESS 0x1E

#define MAGNETOMETER_CONFIG_REGISTER_A 0x00
#define MAGNETOMETER_CONFIG_REGISTER_B 0x01
#define MAGNETOMETER_MODE_REGISTER_ADDRESS 0x02
#define MAGNETOMETER_DATA_OUTPUT_ADDRESS 0x03
#define MAGNETOMETER_ID_ADDRESS 0x0a

#define MAGNETOMETER_GAIN_0 0x00  // 0.88 gauss
#define MAGNETOMETER_GAIN_1 0x20  // 1.3 gauss
#define MAGNETOMETER_GAIN_2 0x40  // 1.9 gauss
#define MAGNETOMETER_GAIN_3 0x60  // 2.5 gauss
#define MAGNETOMETER_GAIN_4 0x80  // 4.0 gauss
#define MAGNETOMETER_GAIN_5 0xA0  // 4.7 gauss
#define MAGNETOMETER_GAIN_6 0xC0  // 5.6 gauss
#define MAGNETOMETER_GAIN_7 0xE0  // 8.1 gauss

#define MAGNETOMETER_MODE_CONTINUOUS_OUTPUT 0x00
#define MAGNETOMETER_MODE_SINGLE_OUTPUT 0x01

typedef struct {
  int x;
  int y;
  int z;
} magnet_t;

void magnetometer_init(void);
void magnetometer_read(magnet_t* coordinates);

#endif
