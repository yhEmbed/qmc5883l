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
#include <eigen3/Eigen/Dense>
#include <iostream>
using namespace Eigen;
using namespace std;
#define I2C_INDEX	1
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
 int I2COpen(unsigned char I2CNum, unsigned char u32DevAddr);
/**
* @brief:	打开I2C。
* @param:	unsigned char I2CNum
* @param:	unsigned char u32DevAddr
* @return:	成功返回GUIDEIR_OK。失败返回GUIDEIR_ERR。
* @note:
**/
 int I2COpen(unsigned char I2CNum, unsigned char u32DevAddr)
{
	char str[128];
	printf("Enter I2COpen I2CNum == %d, u32DevAddr == 0x%x 7bit=0x%x\n", I2CNum, u32DevAddr,u32DevAddr>>1);
	sprintf(str, "/dev/i2c-%d", I2C_INDEX);
	g_Fd = i2c_open(str, u32DevAddr>>1);
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
  qmc5883.rate = QMC5883L_CONFIG_200HZ;
  qmc5883.mode = QMC5883L_CONFIG_CONT;

  
#ifdef I2C_PATH_3
i2c_num = 3;
#endif

#ifdef I2C_PATH_4
i2c_num = 4;
#endif

  if(I2COpen(I2C_INDEX,QMC5883L_ADDR)){
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

typedef struct QMC5883_PARAM
{
	float mx_offset, my_offset, mz_offset;		/*!< 地磁偏移变量 */
	float mx_k,	my_k, mz_k;     				/*!< 地磁标度系数变量 */
}QMC5883_PARAM;

QMC5883_PARAM qmc5883_param;
static char *ini_get_string(char *section, char *key, char *filename)
{
	FILE *fp = NULL;
	char szLine[1024];
	static char tmpstr[1024];
	int flag = 0;
	char *tmp = NULL;
	char *find = NULL;

	fp=fopen(filename, "r");
	if (fp == NULL)
	{
		printf("no such file: %s", filename);
		return NULL;
	}

	while (!feof(fp))
	{	
		memset(szLine, NULL, sizeof(szLine));
		memset(tmpstr, NULL, sizeof(tmpstr));

		/** read a line to szLine */
		fgets(szLine, 1024, fp);

		if ('#' == szLine[0])
		{
			continue;
		}
		else if(szLine[0] == NULL)
		{
			printf("read end\n");
			break;
		}
		else if((isspace(szLine[0])) && (flag == 1))
		{
			printf("no key: %s in section: %s\n", key, section);
			break;
		}
		else
		{
			tmp = strchr(szLine, '=');
		}

		if ((tmp != NULL) && (flag == 1))
		{
			if (strstr(szLine, key) != NULL)
			{
				strcpy(tmpstr, tmp + 1);
				fclose(fp);

				find = strchr(tmpstr, '\n');
				if(find != NULL)
				{
					*find = NULL;
				}
				return tmpstr;
			}
		}
		else
		{
			strcpy(tmpstr, "[");
			strcat(tmpstr, section);
			strcat(tmpstr, "]");
			if (strncmp(tmpstr, szLine, strlen(tmpstr)) == 0)
			{
				//found section
				flag = 1;
				szLine[0] = '\0';
			}
			else
			{
				continue;
			}
		}
	}

	if(flag == 0)
	{
		printf("no such section: %s\n", section);
	}

	fclose(fp);
	return "";
}

static int ini_get_int(char *section, char *key, char *filename)
{
	return atoi(ini_get_string(section, key, filename));
}

// [qmc5883] 
// mx_offset=1285.802368 
// my_offset=574.786072 
// mz_offset=-1325.702393 
// mx_k=4928.951172 
// my_k=5716.808594 
// mz_k=6050.792969 

int qmc5883_save_param(char *file_path)
{
	FILE* fp = NULL;
	char *str;
	int j = 0;
	str = (char *)malloc(1024);
	memset(str, 0, sizeof(1024));
	j = sprintf(str, "[qmc5883] \n", NULL);
	j += sprintf(str+j, "mx_offset=%f \n", qmc5883_param.mx_offset);
	j += sprintf(str+j, "my_offset=%f \n", qmc5883_param.my_offset);
	j += sprintf(str+j, "mz_offset=%f \n", qmc5883_param.mz_offset);
	j += sprintf(str+j, "mx_k=%f \n", qmc5883_param.mx_k);
	j += sprintf(str+j, "my_k=%f \n", qmc5883_param.my_k);
	j += sprintf(str+j, "mz_k=%f \n", qmc5883_param.mz_k);
	printf("str:\n%s", str);
	fp = fopen(file_path, "w+");
	if (fp == NULL)
	{
	    printf("open2222 file %s error\n", file_path);
	    return -1;
	}
	
	fwrite(str, 1, strlen(str), fp);
	fclose(fp);
	return 0;
}

uint8_t qmc5883_get_param(char *file_path)
{

	qmc5883_param.mx_offset = atof(ini_get_string((char *)"qmc5883", (char *)"mx_offset", file_path));
	qmc5883_param.my_offset = atof(ini_get_string((char *)"qmc5883", (char *)"my_offset", file_path));
	qmc5883_param.mz_offset = atof(ini_get_string((char *)"qmc5883", (char *)"mz_offset", file_path));
	qmc5883_param.mx_k = atof(ini_get_string((char *)"qmc5883", (char *)"mx_k", file_path));
	qmc5883_param.my_k = atof(ini_get_string((char *)"qmc5883", (char *)"my_k", file_path));
	qmc5883_param.mz_k = atof(ini_get_string((char *)"qmc5883", (char *)"mz_k", file_path));
	printf("qmc5883_get_param: \n");
	printf("    mx_offset:%f\n", qmc5883_param.mx_offset);
	printf("    my_offset:%f\n", qmc5883_param.my_offset);
	printf("    mz_offset:%f\n", qmc5883_param.mz_offset);
	printf("    mx_k:%f\n", qmc5883_param.mx_k);
	printf("    my_k:%f\n", qmc5883_param.my_k);
	printf("    mz_k:%f\n", qmc5883_param.mz_k);
	// printf("qmc5883_get_param: mx_offset:%s\n", ini_get_string("qmc5883", "mx_offset", file_path));
	// printf("qmc5883_get_param: my_offset:%s\n", ini_get_string("qmc5883", "my_offset", file_path));
	// printf("qmc5883_get_param: mz_offset:%s\n", ini_get_string("qmc5883", "mz_offset", file_path));
	// printf("qmc5883_get_param: mx_k:%s\n", ini_get_string("qmc5883", "mx_k", file_path));
	// printf("qmc5883_get_param: my_k:%s\n", ini_get_string("qmc5883", "my_k", file_path));
	// printf("qmc5883_get_param: mz_k:%s\n", ini_get_string("qmc5883", "mz_k", file_path));
	return 0;
}

/* Macro -----------------------------------------------------------------------------------*/
/* Typedef ---------------------------------------------------------------------------------*/
enum
{
	AXIS_X = 0,
	AXIS_Y = 1,
	AXIS_Z = 2,

	AXIS_TOTAL
};


/* Macro -----------------------------------------------------------------------------------*/
/* Typedef ---------------------------------------------------------------------------------*/
/* Variables -------------------------------------------------------------------------------*/
float mx, my, mz;


unsigned char  g_qmc5883_calculate_flag = 0;

typedef struct {
    float XY_Angle;
	  float XZ_Angle;
	  float YZ_Angle;
}MagnetAngle_t;

MagnetAngle_t Sample_Angle;

uint8_t qmc5883_get_data(char verbose)
{
	uint8_t Buff[6], i;

	short MagnetRawAd[3];

	while(!qmc5883_ready()) {
		usleep(10);}

  	qmc5883_read(QMC5883L_X_LSB,Buff,6);

	//printf("Xl=%d, Xh=%d, Yl=%d, Yh=%d, Zl=%d, Zh=%d \r\n",Buff[0],Buff[1],Buff[2],Buff[3],Buff[4],Buff[5]);
	
	MagnetRawAd[AXIS_X] = -(short)((Buff[1] << 8) | Buff[0]);
	MagnetRawAd[AXIS_Y] = (short)((Buff[3] << 8) | Buff[2]);
	MagnetRawAd[AXIS_Z] = (short)((Buff[5] << 8) | Buff[4]);

	//printf("MagnetRawAd X:%d, X:%d, X:%d\r\n",MagnetRawAd[AXIS_X], MagnetRawAd[AXIS_Y], MagnetRawAd[AXIS_Z]);

	if(	g_qmc5883_calculate_flag==1)
	{
		mx=(MagnetRawAd[AXIS_X]-qmc5883_param.mx_offset)/qmc5883_param.mx_k;
		my=(MagnetRawAd[AXIS_Y]-qmc5883_param.my_offset)/qmc5883_param.my_k;
		mz=(MagnetRawAd[AXIS_Z]-qmc5883_param.mz_offset)/qmc5883_param.mz_k;
		if(verbose) printf("mx=%f,my=%f,mz=%f \r\n",mx,my,mz);
	}
	else
	{
		mx=MagnetRawAd[AXIS_X];
		my=MagnetRawAd[AXIS_Y];
		mz=MagnetRawAd[AXIS_Z];
	}	
		
	#if 0
	MagnetRawAd[0] = ((int16_t)QMC5883L_ReadReg(QMC5883L_ADDR_XOUTH)<<8) | QMC5883L_ReadReg(QMC5883L_ADDR_XOUTL);
	MagnetRawAd[1] = ((int16_t)QMC5883L_ReadReg(QMC5883L_ADDR_YOUTH)<<8) | QMC5883L_ReadReg(QMC5883L_ADDR_YOUTL);
	MagnetRawAd[2] = ((int16_t)QMC5883L_ReadReg(QMC5883L_ADDR_ZOUTH)<<8) | QMC5883L_ReadReg(QMC5883L_ADDR_ZOUTL);
	#endif
	
	return 1;
}

/**
 * @brief 	
 * @param 	
 * @return 
 * @details 
 */
void qmc5883_convert_rawdata(char verbose)
{
	float magGauss_norm = 0;

	// 磁场强度
	magGauss_norm = sqrt(mx * mx + my * my + mz * mz);
	if(verbose) printf("magGauss_norm =%f\r\n",magGauss_norm);

	//方向角计算：方向角是X轴和Y轴读数的反正切 
	// X轴指向正北时，XY_Angle=0
	// Sample_Angle.XY_Angle = (atan2((float)mx,(float)my) * (180 / 3.14159265) + 180);  
	// Sample_Angle.XZ_Angle = (atan2((float)mz,(float)mx) * (180 / 3.14159265) + 180);
	// Sample_Angle.YZ_Angle = (atan2((float)mz,(float)my) * (180 / 3.14159265) + 180);	
	// Y轴指向正北时，XY_Angle=0(UTx318M地磁传感器Y轴指向设备正前方)
	Sample_Angle.XY_Angle = - (atan2((float)mx,(float)my) * (180 / 3.14159265));  
	Sample_Angle.XZ_Angle = (atan2((float)mz,(float)mx) * (180 / 3.14159265) + 180);
	Sample_Angle.YZ_Angle = (atan2((float)mz,(float)my) * (180 / 3.14159265) + 180);		
	if(Sample_Angle.XY_Angle < 0) Sample_Angle.XY_Angle +=360;
	if(verbose) printf("Angle data :XY_Angle=%f,XZ_Angle=%f,YZ_Angle=%f\r\n",Sample_Angle.XY_Angle, Sample_Angle.XZ_Angle,Sample_Angle.YZ_Angle);
		
//  HAL_Delay(500);
}


//椭球校准
float 	x_sum=0,    xx_sum=0,   xxx_sum=0,  yyyy_sum=0,
		y_sum=0,    yy_sum=0,   xxy_sum=0,  zzzz_sum=0,
		z_sum=0,    zz_sum=0,   xxz_sum=0,  xxyy_sum=0,
					xy_sum=0,   xyy_sum=0,  xxzz_sum=0,
					xz_sum=0,   xzz_sum=0,  yyzz_sum=0,
					yz_sum=0,   yyy_sum=0,
								yyz_sum=0,
								yzz_sum=0,
								zzz_sum=0;

float  	x_avr=0,    xx_avr=0,   xxx_avr=0,  yyyy_avr=0,
		y_avr=0,    yy_avr=0,   xxy_avr=0,  zzzz_avr=0,
		z_avr=0,    zz_avr=0,   xxz_avr=0,  xxyy_avr=0,
					xy_avr=0,   xyy_avr=0,  xxzz_avr=0,
					xz_avr=0,   xzz_avr=0,  yyzz_avr=0,
					yz_avr=0,   yyy_avr=0,
								yyz_avr=0,
								yzz_avr=0,
								zzz_avr=0;

/**
 * @brief   :椭球拟合
 * @param   :  
 * @retva   : 
 */
char calculate_qmc5883(uint32_t calculate_times)
{
	static uint16_t n=0;
	char ret = -1;
	if(g_qmc5883_calculate_flag==2)//各次累加和统计
	{
		n++;
		x_sum+=mx; xx_sum+=(mx*mx); xxx_sum+=(mx*mx*mx); yyyy_sum+=(my*my*my*my);
		y_sum+=my; yy_sum+=(my*my); xxy_sum+=(mx*mx*my); zzzz_sum+=(mz*mz*mz*mz);
		z_sum+=mz; zz_sum+=(mz*mz); xxz_sum+=(mx*mx*mz); xxyy_sum+=(mx*mx*my*my);
		           xy_sum+=(mx*my); xyy_sum+=(mx*my*my); xxzz_sum+=(mx*mx*mz*mz);
		           xz_sum+=(mx*mz); xzz_sum+=(mx*mz*mz); yyzz_sum+=(my*my*mz*mz);
		           yz_sum+=(my*mz); yyy_sum+=(my*my*my);
		                            yyz_sum+=(my*my*mz);
		                            yzz_sum+=(my*mz*mz);
		                            zzz_sum+=(mz*mz*mz);
		
		if(n>calculate_times){
			g_qmc5883_calculate_flag=3;
		}
		if(n % 100 == 0){
			printf("Calculate_QMC5883 n=%d \n", n);
		}
		ret = 0;
	}
	else if(g_qmc5883_calculate_flag==3)
	{
		float                   A[36]   ,      A_inv [36];
		MatrixXd A_matrix(6,6), A_inv_matrix(6,6);
    	float                   B[6];
   		MatrixXd B_matrix(6,1);
		float                   Par[6];//需要拟合的参数
		MatrixXd Par_matrix(6,1);
		
		
		//各次均值统计
		x_avr=x_sum/n; xx_avr=xx_sum/n; xxx_avr=xxx_sum/n; yyyy_avr=yyyy_sum/n;
		y_avr=y_sum/n; yy_avr=yy_sum/n; xxy_avr=xxy_sum/n; zzzz_avr=zzzz_sum/n;
		z_avr=z_sum/n; zz_avr=zz_sum/n; xxz_avr=xxz_sum/n; xxyy_avr=xxyy_sum/n;
		               xy_avr=xy_sum/n; xyy_avr=xyy_sum/n; xxzz_avr=xxzz_sum/n;
		               xz_avr=xz_sum/n; xzz_avr=xzz_sum/n; yyzz_avr=yyzz_sum/n;
		               yz_avr=yz_sum/n; yyy_avr=yyy_sum/n;
		                                yyz_avr=yyz_sum/n;
		                                yzz_avr=yzz_sum/n;
		                                zzz_avr=zzz_sum/n;
		
		//系数矩阵赋值
		A[0 ]=yyyy_avr;A[1 ]=yyzz_avr;A[2 ]=xyy_avr;A[3 ]=yyy_avr;A[4 ]=yyz_avr; A[5 ]=yy_avr;
		A[6 ]=yyzz_avr;A[7 ]=zzzz_avr;A[8 ]=xzz_avr;A[9 ]=yzz_avr;A[10]=zzz_avr; A[11]=zz_avr;
		A[12]=xyy_avr ;A[13]=xzz_avr ;A[14]=xx_avr ;A[15]=xy_avr ;A[16]=xz_avr ; A[17]=x_avr ;
		A[18]=yyy_avr ;A[19]=yzz_avr ;A[20]=xy_avr ;A[21]=yy_avr ;A[22]=yz_avr ; A[23]=y_avr ;
		A[24]=yyz_avr ;A[25]=zzz_avr ;A[26]=xz_avr ;A[27]=yz_avr ;A[28]=zz_avr ; A[29]=z_avr ;
		A[30]=yy_avr  ;A[31]=zz_avr  ;A[32]=x_avr  ;A[33]=y_avr  ;A[34]=z_avr  ; A[35]=1     ;
    
		A_matrix << A[0 ],A[1 ],A[2 ],A[3 ],A[4 ],A[5 ],\
				A[6 ],A[7 ],A[8 ],A[9 ],A[10],A[11],\
				A[12],A[13],A[14],A[15],A[16],A[17],\
				A[18],A[19],A[20],A[21],A[22],A[23],\
				A[24],A[25],A[26],A[27],A[28],A[29], \
				A[30],A[31],A[32],A[33],A[34],A[35];
		
		if(verbose) cout << "A_matrix=\n" << A_matrix << endl;
		if(verbose) cout << "A_matrix.inverse=\n" << A_matrix.inverse() << endl;
		//系数矩阵求逆
		A_inv_matrix = A_matrix.inverse();
		if(verbose) cout << "A_inv_matrix=\n" << A_inv_matrix << endl;
			//非齐次列向量赋值
		B[0]=-xxyy_avr;
		B[1]=-xxzz_avr;
		B[2]=-xxx_avr ;
		B[3]=-xxy_avr ;
		B[4]=-xxz_avr ;
		B[5]=-xx_avr  ;
		B_matrix << B[0],B[1],B[2],B[3],B[4],B[5];
		if(verbose) cout << "B_matrix=\n" << B_matrix << endl;
			
			//解方程组得出拟合参数
		Par_matrix = A_inv_matrix*B_matrix;
		if(verbose) cout << "Par_matrix=\n" << Par_matrix << endl;
			//计算椭球参数
			qmc5883_param.mx_offset=(-Par_matrix(2)/2.0f);             //拟合出的x轴中心坐标
		qmc5883_param.my_offset=(-Par_matrix(3))/(2.0f*Par_matrix(0));    //拟合出的y轴中心坐标
		qmc5883_param.mz_offset=(-Par_matrix(4)/(2.0f*Par_matrix(1)));    //拟合出的z轴中心坐标
		if(verbose) printf("mx_offset=%f,my_offset=%f,mz_offset=%f \r\n",qmc5883_param.mx_offset,qmc5883_param.my_offset,qmc5883_param.mz_offset);
    
		qmc5883_param.mx_k= sqrtf(qmc5883_param.mx_offset*qmc5883_param.mx_offset + Par_matrix(0)*qmc5883_param.my_offset*qmc5883_param.my_offset + Par_matrix(1)*qmc5883_param.mz_offset*qmc5883_param.mz_offset - Par_matrix(5));  //拟合出的x方向上的轴半径
		qmc5883_param.my_k= qmc5883_param.mx_k/sqrt(Par_matrix(0));                                                                              //拟合出的y方向上的轴半径
		qmc5883_param.mz_k= qmc5883_param.mx_k/sqrt(Par_matrix(1));                                                                              //拟合出的z方向上的轴半径
		if(verbose) printf("mx_k=%f,my_k=%f,mz=%f \r\n",qmc5883_param.mx_k,qmc5883_param.my_k,qmc5883_param.mz_k);
		
		// g_qmc5883_calculate_flag=1;

		ret = 1;
	}		
	return ret;
}

void qmc5883_dump_angle2file(const char* file_path)
{
	char str[128] = {0};
	sprintf(str, "%d", (int)Sample_Angle.XY_Angle);
	FILE* fp = fopen(file_path, "w");
	if (fp == NULL)
	{
	    printf("open file %s error\n", file_path);
	    return;
	}
	fwrite(str, strlen(str), 1, fp);
	fclose(fp);
	//printf("save to file %s success\n", file_path);
}

void qmc5883_resetCalibration() {
  qmc5883.xhigh = qmc5883.yhigh = 0;
  qmc5883.xlow = qmc5883.ylow = 0;
}

int qmc5883_read_heading()
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

