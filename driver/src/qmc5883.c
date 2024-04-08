#include <stdio.h>
#include <stdarg.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <unistd.h>
#include <pthread.h>
//#include "hal_gpio.h"
#include "i2c.h"
#include <math.h>
#include "qmc5883.h"
#define I2C_PATH_3
// #define I2C_PATH_4
/* The default I2C address of this chip */
#define QMC5883L_ADDR (0x0D<<1)

/* Register numbers */
#define QMC5883L_X_LSB 0
#define QMC5883L_X_MSB 1
#define QMC5883L_Y_LSB 2
#define QMC5883L_Y_MSB 3
#define QMC5883L_Z_LSB 4
#define QMC5883L_Z_MSB 5
#define QMC5883L_STATUS 6
#define QMC5883L_TEMP_LSB 7
#define QMC5883L_TEMP_MSB 8
#define QMC5883L_CONFIG 9
#define QMC5883L_CONFIG2 10
#define QMC5883L_RESET 11
#define QMC5883L_RESERVED 12
#define QMC5883L_CHIP_ID 13

/* Bit values for the STATUS register */
#define QMC5883L_STATUS_DRDY 1
#define QMC5883L_STATUS_OVL 2
#define QMC5883L_STATUS_DOR 4

/* Oversampling values for the CONFIG register */
#define QMC5883L_CONFIG_OS512 0b00000000
#define QMC5883L_CONFIG_OS256 0b01000000
#define QMC5883L_CONFIG_OS128 0b10000000
#define QMC5883L_CONFIG_OS64  0b11000000

/* Range values for the CONFIG register */
#define QMC5883L_CONFIG_2GAUSS 0b00000000
#define QMC5883L_CONFIG_8GAUSS 0b00010000

/* Rate values for the CONFIG register */
#define QMC5883L_CONFIG_10HZ   0b00000000
#define QMC5883L_CONFIG_50HZ   0b00000100
#define QMC5883L_CONFIG_100HZ  0b00001000
#define QMC5883L_CONFIG_200HZ  0b00001100

/* Mode values for the CONFIG register */
#define QMC5883L_CONFIG_STANDBY 0b00000000
#define QMC5883L_CONFIG_CONT    0b00000001

/* Apparently M_PI isn't available in all environments. */
#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif
#ifdef I2C_PATH_4
#define DEV_I2C_PATH ("/dev/i2c-4")
#endif

#ifdef I2C_PATH_3
#define DEV_I2C_PATH ("/dev/i2c-3")
#endif

static  int g_Fd = 0;

static struct QMC5883L  qmc5883;

/**
* @brief:	打开I2C。
* @param:	unsigned char I2CNum
* @param:	unsigned char u32DevAddr
* @return:	成功返回GUIDEIR_OK。失败返回GUIDEIR_ERR。
* @note:
**/
 int I2COpen(unsigned char I2CNum, unsigned char u32DevAddr)
{
	printf("Enter I2COpen I2CNum == %d, u32DevAddr == 0x%x 7bit=0x%x\n", I2CNum, u32DevAddr,u32DevAddr>>1);
	
	g_Fd = i2c_open(DEV_I2C_PATH, u32DevAddr>>1);
	if (g_Fd < 0) {
		return -1;
	}

	return 0;
}

/**
* @brief:	关闭I2C。
* @param:	unsigned char I2CNum
* @return:	成功返回GUIDEIR_OK。失败返回GUIDEIR_ERR。
* @note:
**/
 int I2CClose(unsigned char I2CNum)
{
	printf("Enter I2CClose I2CNum == %d\n", I2CNum);
	i2c_close(g_Fd);
	//i2c_stop(&g_I2cDev);
	return 0;
}

/**
* @brief:	I2C读寄存器。
* @param:	unsigned char u8Addr	寄存器地址。
* @param:	unsigned char * pu8Val	读出的寄存器值。
* @return:	成功返回GUIDEIR_OK。失败返回GUIDEIR_ERR。
* @note:
**/
 int I2CRead(unsigned char u8Addr, unsigned char * pu8Val)
{
	int rc;
	rc = i2c_read(g_Fd,u8Addr,pu8Val);
	if(rc<0){
		return -1;
	}

	//printf("I2CRead OK, u8Addr == 0x%x, pu8Val == 0x%x\n", u8Addr, *pu8Val);
	return 0;
}

/**
* @brief:	I2C写寄存器。
* @param:	unsigned char u8Addr	寄存器地址。
* @param:	unsigned char u8Val		写入的寄存器值。
* @return:	成功返回GUIDEIR_OK。失败返回GUIDEIR_ERR。
* @note:
**/
 int I2CWrite(unsigned char u8Addr, unsigned char u8Val)
{

	//printf("I2CWrite OK, u8Addr == 0x%x, u8Val == 0x%x\n", u8Addr, u8Val);

	int rc;
	//rc=i2c_write_reg(&g_I2cDev,u8Addr,u8Val);
	rc = i2c_write(g_Fd,u8Addr,u8Val);
	if(rc<0){
		return -1;
	}

	//printf("I2CWrite OK, u8Addr == 0x%x, u8Val == 0x%x\n", u8Addr, u8Val);
	return 0;
}

void qmc5883_read(uint8_t start_reg_addr, uint8_t *pdata, uint8_t size){
  int i=0;
  for(i=0;i<size;i++){
	  if(I2CRead(start_reg_addr,pdata))printf("qmc5883_I2C_Read failed\n");
	  start_reg_addr++;
	  pdata++;
	}
}

void qmc5883_write(uint8_t start_reg_addr, uint8_t *pdata, uint8_t size){
 	int i=0;
	for(i=0;i<size;i++){
	  if(I2CWrite(start_reg_addr,*pdata))printf("qmc5883_I2C_Write failed\n");
	  start_reg_addr++;
	  pdata++;
	}
}


void qmc5883_reconfig()
{
	uint8_t val;
  	
  	val=qmc5883.oversampling|qmc5883.range|qmc5883.rate|qmc5883.mode;
	qmc5883_write(QMC5883L_CONFIG,&val,1);
  val = 0x40;
  qmc5883_write(0x20,&val,1);
  val = 0x01;
  qmc5883_write(0x21,&val,1);
}

void qmc5883_reset()
{
  uint8_t val;
  
  val=1;
  qmc5883_write(QMC5883L_RESET,&val,1);
  qmc5883_reconfig();
}

void qmc5883_setOversampling( int x )
{
  switch(x) {
    case 512:
      qmc5883.oversampling = QMC5883L_CONFIG_OS512;
      break;
    case 256:
      qmc5883.oversampling = QMC5883L_CONFIG_OS256;
      break;
    case 128:
      qmc5883.oversampling = QMC5883L_CONFIG_OS128;
      break;
    case 64:
      qmc5883.oversampling = QMC5883L_CONFIG_OS64;
      break;
  } 
  qmc5883_reconfig();
}

void qmc5883_setRange( int x )
{
  switch(x) {
    case 2:
      qmc5883.range = QMC5883L_CONFIG_2GAUSS;
      break;
    case 8:
      qmc5883.range = QMC5883L_CONFIG_8GAUSS;
      break;
  }
  qmc5883_reconfig();
}

void qmc5883_setSamplingRate( int x )
{
  switch(x) {
    case 10:
      qmc5883.rate = QMC5883L_CONFIG_10HZ;
      break;
    case 50:
      qmc5883.rate = QMC5883L_CONFIG_50HZ;
      break;
    case 100:
      qmc5883.rate = QMC5883L_CONFIG_100HZ;
      break;
    case 200:
      qmc5883.rate = QMC5883L_CONFIG_200HZ;
      break;
  }
  qmc5883_reconfig();
}

int qmc5883_init() {
  int i2c_num = 0;
  /* This assumes the wire library has been initialized. */
  qmc5883.addr = QMC5883L_ADDR;
  qmc5883.oversampling = QMC5883L_CONFIG_OS512;
  qmc5883.range = QMC5883L_CONFIG_2GAUSS;
  qmc5883.rate = QMC5883L_CONFIG_50HZ;
  qmc5883.mode = QMC5883L_CONFIG_CONT;

  
#ifdef I2C_PATH_3
i2c_num = 3;
#endif

#ifdef I2C_PATH_4
i2c_num = 4;
#endif

  if(I2COpen(i2c_num,QMC5883L_ADDR)){
  	printf("QMC5883_init I2COpen failed\n");
	return -1;
  }

  qmc5883_reset();
  return 0;
}

int qmc5883_ready()
{
  //if(!read_register(addr,QMC5883L_STATUS,1)) return 0;
  //uint8_t status = Wire.read();
  uint8_t status;
  qmc5883_read(QMC5883L_STATUS,&status,1);
  return status & QMC5883L_STATUS_DRDY; 
}

int qmc5883_readRaw( int16_t *x, int16_t *y, int16_t *z, int16_t *t )
{

  uint8_t buf[6];
  int16_t raw[3];
  while(!qmc5883_ready()) {
  	usleep(10);}

  qmc5883_read(QMC5883L_X_LSB,buf,6);
  raw[0] = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);                                         /* get x raw */
  raw[1] = (int16_t)(((uint16_t)buf[3] << 8) | buf[2]);                                         /* get y raw */
  raw[2] = (int16_t)(((uint16_t)buf[5] << 8) | buf[4]);                                         /* get z raw */

	
  *x = raw[0];
  *y = raw[1];
  *z = raw[2];
  

  return 1;
}

void qmc5883_resetCalibration() {
  qmc5883.xhigh = qmc5883.yhigh = 0;
  qmc5883.xlow = qmc5883.ylow = 0;
}

int qmc5883_readHeading()
{
  int16_t x, y, z, t;

  if(!qmc5883_readRaw(&x,&y,&z,&t)) return 0;

  /* Update the observed boundaries of the measurements */

  if(x<qmc5883.xlow) qmc5883.xlow = x;
  if(x>qmc5883.xhigh) qmc5883.xhigh = x;
  if(y<qmc5883.ylow) qmc5883.ylow = y;
  if(y>qmc5883.yhigh) qmc5883.yhigh = y;

  /* Bail out if not enough data is available. */
  
  if( qmc5883.xlow==qmc5883.xhigh || qmc5883.ylow==qmc5883.yhigh ) return 0;

  /* Recenter the measurement by subtracting the average */

  x -= (qmc5883.xhigh+qmc5883.xlow)/2;
  y -= (qmc5883.yhigh+qmc5883.ylow)/2;

  /* Rescale the measurement to the range observed. */
  
  float fx = (float)x/(qmc5883.xhigh-qmc5883.xlow);
  float fy = (float)y/(qmc5883.yhigh-qmc5883.ylow);

  int heading = 180.0*atan2(fy,fx)/M_PI;
  int heading2 = 180.0*atan2(y,x)/M_PI;
  
  if(heading<=0) heading += 360;
  if(heading2<=0) heading2 += 360;
  printf("XY_Angle2=%03d,    ",heading2);
  
  return heading;
}

