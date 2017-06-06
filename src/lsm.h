#ifndef LSM_H
#define LSM_H

typedef struct {
  int ax, ay, az;
  int gx, gy, gz;
} lsm_t;

bool lsm_init();
void lsm_sample(lsm_t *sample);

#endif // LSM_H
