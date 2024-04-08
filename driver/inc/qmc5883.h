#ifndef QMC5883L_H
#define QMC5883L_H
#include <stdint.h>

  int qmc5883_init();
  void qmc5883_reset();
  int  qmc5883_ready();
  void qmc5883_reconfig();
  int  qmc5883_readHeading();
  int  qmc5883_readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
  void qmc5883_resetCalibration();
  void qmc5883_setSamplingRate( int rate );
  void qmc5883_setRange( int range );
  void qmc5883_setOversampling( int ovl );
struct QMC5883L {
  int16_t xhigh, xlow;
  int16_t yhigh, ylow;
  uint8_t addr;
  uint8_t mode;
  uint8_t rate;
  uint8_t range;
  uint8_t oversampling;
};

#endif
