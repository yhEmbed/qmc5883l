#ifndef QMC5883L_H
#define QMC5883L_H
#include <stdint.h>

  int qmc5883_init();
  void qmc5883_reset();
  int  qmc5883_ready();
  void qmc5883_reconfig();
  int  qmc5883_read_heading();
  int  qmc5883_readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t );
  void qmc5883_resetCalibration();
  void qmc5883_setSamplingRate( int rate );
  void qmc5883_setRange( int range );
  void qmc5883_setOversampling( int ovl );
  int qmc5883_save_param(char *file_path);
  char calculate_qmc5883(uint32_t calculate_times, char verbose);
  void qmc5883_convert_rawdata(char verbose);
  uint8_t qmc5883_get_data(char verbose);
  uint8_t qmc5883_get_param(char *file_path);
  void qmc5883_dump_angle2file(const char* file_path);
/* Variables -------------------------------------------------------------------------------*/
extern unsigned char  g_qmc5883_calculate_flag;
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
